from __future__ import annotations

import os
import time
import logging
from datetime import datetime
from dataclasses import dataclass
from threading import Event, Thread
from queue import Queue, Empty
from typing import Optional

try:
    import cv2  # type: ignore
except ImportError:  # pragma: no cover
    cv2 = None  # type: ignore

try:
    import boto3  # type: ignore
    from botocore.exceptions import BotoCoreError, ClientError  # type: ignore
except ImportError:  # pragma: no cover
    boto3 = None  # type: ignore
    BotoCoreError = ClientError = Exception  # type: ignore

LOGGER = logging.getLogger(__name__)


@dataclass(slots=True)
class ImageCaptureConfig:
    root_dir: str
    capture_interval: float
    camera_index: int
    jpeg_quality: int
    upload: bool
    bucket: str
    prefix: str
    remove_after_upload: bool


def load_image_capture_config(node, *, vehicle_id: str) -> ImageCaptureConfig:
    default_capture_root = os.path.join(os.path.dirname(__file__), "uav_captures")
    root_dir = str(node.declare_parameter("image_capture_root", default_capture_root).value)
    interval = float(node.declare_parameter("image_capture_interval_sec", 5.0).value)
    camera_index = int(node.declare_parameter("image_capture_camera_index", 0).value)
    jpeg_quality = int(node.declare_parameter("image_capture_jpeg_quality", 90).value)
    upload = bool(node.declare_parameter("image_capture_upload", True).value)
    bucket = str(
        node.declare_parameter("image_capture_bucket", "amzn-uav-control-image-stream-1").value
    )
    prefix = str(node.declare_parameter("image_capture_prefix", vehicle_id).value)
    remove_after_upload = bool(
        node.declare_parameter("image_capture_remove_after_upload", False).value #giu lai anh sau khi upload
    )

    return ImageCaptureConfig(
        root_dir=root_dir,
        capture_interval=interval,
        camera_index=camera_index,
        jpeg_quality=jpeg_quality,
        upload=upload,
        bucket=bucket,
        prefix=prefix,
        remove_after_upload=remove_after_upload,
    )


class ImageCaptureManager:
    """
    Background helper that periodically captures images from a local camera and (optionally)
    uploads them to an S3 bucket. Each capture session owns its own folder, named using the
    latest GPS fix to make post-flight correlation easier.
    """

    def __init__(
        self,
        *,
        root_dir: str,
        capture_interval: float = 5.0,
        camera_index: int = 0,
        jpeg_quality: int = 90,
        upload_to_s3: bool = True,
        bucket_name: str = "",
        s3_prefix: str = "",
        remove_after_upload: bool = True,
        logger: Optional[logging.Logger] = None,
    ) -> None:
        if cv2 is None:
            raise RuntimeError("OpenCV (cv2) is required for image capture mode")

        self.root_dir = root_dir
        self.capture_interval = max(0.1, float(capture_interval))
        self.camera_index = int(camera_index)
        self.jpeg_quality = max(10, min(100, int(jpeg_quality)))
        self.upload_to_s3 = bool(upload_to_s3)
        self.bucket_name = bucket_name
        self.s3_prefix = s3_prefix.strip().strip("/")
        self.remove_after_upload = bool(remove_after_upload)
        self._logger = logger or LOGGER

        self._capture_thread: Optional[Thread] = None
        self._upload_thread: Optional[Thread] = None
        self._stop_event: Optional[Event] = None
        self._upload_stop: Optional[Event] = None
        self._queue: Optional[Queue[str]] = None
        self._session_name: Optional[str] = None
        self._session_dir: Optional[str] = None
        self._cap: Optional["cv2.VideoCapture"] = None
        self._captured_count = 0
        self._uploaded_count = 0
        self._s3_client = None
        self._session_upload_enabled = False

    @property
    def is_running(self) -> bool:
        return self._capture_thread is not None and self._capture_thread.is_alive()

    def start_session(
        self,
        *,
        lat: Optional[float] = None,
        lon: Optional[float] = None,
        alt: Optional[float] = None,
    ) -> bool:
        """
        Start a new capture session. Returns True if the session started successfully.
        """
        if self.is_running:
            self._logger.debug("Image capture session already running → skip start")
            return True

        self._ensure_root_dir()
        session_name = self._build_session_name(lat, lon, alt)
        session_dir = os.path.join(self.root_dir, session_name)
        try:
            os.makedirs(session_dir, exist_ok=True)
        except OSError as exc:
            self._logger.error(f"Failed to create capture dir '{session_dir}': {exc}")
            return False

        cap = cv2.VideoCapture(self.camera_index)
        if not cap.isOpened():
            self._logger.error(f"Cannot open camera index {self.camera_index}")
            return False

        self._session_name = session_name
        self._session_dir = session_dir
        self._cap = cap
        self._stop_event = Event()
        self._queue = Queue()
        self._upload_stop = Event()
        self._captured_count = 0
        self._uploaded_count = 0

        self._session_upload_enabled = self.upload_to_s3 and boto3 is not None

        self._capture_thread = Thread(target=self._capture_loop, daemon=True)
        self._capture_thread.start()

        if self._session_upload_enabled:
            self._upload_thread = Thread(target=self._upload_worker, daemon=True)
            self._upload_thread.start()
        elif self.upload_to_s3 and boto3 is None:
            self._logger.warning("boto3 is unavailable → S3 upload disabled for this session")

        self._logger.info(
            f"Image capture session started at {session_name} (interval={self.capture_interval}s)"
        )
        return True

    def stop_session(self) -> int:
        """
        Stop the current capture session. Returns the number of images that were
        successfully flushed (uploaded or stored locally).
        """
        if not self.is_running:
            return 0

        assert self._stop_event is not None
        self._stop_event.set()
        if self._capture_thread:
            self._capture_thread.join()
        self._capture_thread = None

        if self._cap:
            self._cap.release()
            self._cap = None

        total_sent = 0

        if self._session_upload_enabled and self._queue is not None:
            self._queue.join()
            assert self._upload_stop is not None
            self._upload_stop.set()
            if self._upload_thread:
                self._upload_thread.join()
            total_sent = self._uploaded_count
        else:
            total_sent = self._captured_count

        self._cleanup_session()
        self._logger.info(f"Image capture session stopped after {total_sent} image(s)")
        return total_sent

    def shutdown(self) -> None:
        """Stop any running session without raising."""
        try:
            self.stop_session()
        except Exception as exc:  # pragma: no cover - defensive
            self._logger.error(f"Failed to stop image capture session cleanly: {exc}")

    # ----- Internal helpers -------------------------------------------------
    def _ensure_root_dir(self) -> None:
        os.makedirs(self.root_dir, exist_ok=True)

    def _build_session_name(
        self,
        lat: Optional[float],
        lon: Optional[float],
        alt: Optional[float],
    ) -> str:
        lat_part = f"{lat:.6f}" if lat is not None else "lat_unknown"
        lon_part = f"{lon:.6f}" if lon is not None else "lon_unknown"
        alt_part = f"{alt:.1f}" if alt is not None else "alt_unknown"
        timestamp = datetime.now().strftime("%Y%m%dT%H%M%S")
        return f"{lat_part}_{lon_part}_{alt_part}_{timestamp}"

    def _capture_loop(self) -> None:
        assert self._cap is not None
        assert self._stop_event is not None
        while not self._stop_event.is_set():
            loop_start = time.time()
            ok, frame = self._cap.read()
            if not ok:
                self._logger.warning("Failed to read frame from camera; retrying in 0.1s")
                time.sleep(0.1)
                continue

            filename = datetime.now().strftime("%H%M%S_%f")[:-3] + ".jpg"
            assert self._session_dir is not None
            file_path = os.path.join(self._session_dir, filename)
            try:
                success = cv2.imwrite(
                    file_path,
                    frame,
                    [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
                )
            except Exception as exc:  # pragma: no cover - cv2 level errors
                self._logger.error(f"Failed to write image: {exc}")
                success = False

            if success:
                self._captured_count += 1
                if self._session_upload_enabled and self._queue is not None:
                    self._queue.put(file_path)
                else:
                    self._uploaded_count += 1
                    if self.remove_after_upload and self._session_upload_enabled:
                        try:
                            os.remove(file_path)
                        except OSError as exc:
                            self._logger.warning(f"Cannot remove image {file_path}: {exc}")

            elapsed = time.time() - loop_start
            sleep_time = self.capture_interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _upload_worker(self) -> None:
        assert self._queue is not None
        assert self._upload_stop is not None
        while not self._upload_stop.is_set() or not self._queue.empty():
            try:
                file_path = self._queue.get(timeout=0.5)
            except Empty:
                continue

            try:
                if self._upload_single(file_path):
                    self._uploaded_count += 1
                    if self.remove_after_upload:
                        try:
                            os.remove(file_path)
                        except OSError as exc:
                            self._logger.warning(f"Cannot remove uploaded file {file_path}: {exc}")
                else:
                    self._logger.warning(
                        f"Failed to upload {os.path.basename(file_path)}; retry in 5s"
                    )
                    time.sleep(5)
                    self._queue.put(file_path)
            finally:
                self._queue.task_done()

    def _upload_single(self, file_path: str) -> bool:
        if boto3 is None:
            return False
        if not self.bucket_name:
            self._logger.error("Bucket name is empty; cannot upload images")
            return False
        try:
            if self._s3_client is None:
                self._s3_client = boto3.client("s3")
            rel_name = os.path.basename(file_path)
            session_prefix = self._session_name or "session"
            key_parts = [self.s3_prefix] if self.s3_prefix else []
            key_parts.extend([session_prefix, rel_name])
            s3_key = "/".join(key_parts)
            self._s3_client.upload_file(
                Filename=file_path,
                Bucket=self.bucket_name,
                Key=s3_key,
                ExtraArgs={"ContentType": "image/jpeg"},
            )
            self._logger.info(
                f"Uploaded {rel_name} to s3://{self.bucket_name}/{s3_key}"
            )
            return True
        except (BotoCoreError, ClientError) as exc:
            self._logger.error(f"S3 upload error: {exc}")
            return False
        except Exception as exc:  # pragma: no cover - boto3 internals
            self._logger.error(f"Unexpected upload error: {exc}")
            return False

    def _cleanup_session(self) -> None:
        self._session_name = None
        self._session_dir = None
        self._queue = None
        self._upload_thread = None
        self._upload_stop = None
        self._stop_event = None
        self._captured_count = 0
        self._uploaded_count = 0
        self._session_upload_enabled = False


__all__ = ["ImageCaptureConfig", "ImageCaptureManager", "load_image_capture_config"]


if __name__ == "__main__":  # pragma: no cover - manual testing helper
    logging.basicConfig(level=logging.INFO, format="[IMAGE_CAPTURE] %(message)s")
    manager = ImageCaptureManager(
        root_dir=os.path.join(os.getcwd(), "uav_captures_demo"),
        capture_interval=5.0,
        upload_to_s3=False,
    )
    manager.start_session(lat=10.0, lon=106.0, alt=50.0)
    try:
        time.sleep(20)
    finally:
        sent = manager.stop_session()
        print(f"Captured {sent} image(s)")
