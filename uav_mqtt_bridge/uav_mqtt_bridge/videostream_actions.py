from __future__ import annotations

import subprocess
from dataclasses import dataclass
from typing import Optional, Any, Callable

"""
Các helper liên quan đến video stream để gimbal.py (hoặc node khác) có thể bật/tắt
FFmpeg/payload một cách tập trung.
"""

_VIDEO_PROCESS: Optional[subprocess.Popen] = None


@dataclass(slots=True)
class PayloadStreamConfig:
    video_value: str
    video_command: str
    video_use_terminal: bool
    video_terminal_program: str


@dataclass(slots=True)
class StreamConfig:
    mode: str  # "video_stream"
    stream: PayloadStreamConfig


def load_payload_config(node: Any) -> StreamConfig:
    """
    Helper gói toàn bộ phần declare_parameter của video stream để việc chỉnh
    sửa sau này chỉ cần đụng đến videostream_actions.
    """

    mode = str(
        node.declare_parameter("payload_mode", "video_stream").value
    ).strip().lower()

    video_value = node.declare_parameter(
        "video_stream_value", "rtsp://34.56.241.85:8554/a8"
    ).value
    video_command = node.declare_parameter(
        # "video_stream_command",
        # "ffmpeg -hide_banner -nostdin -loglevel warning "
        # "-rtsp_transport tcp -use_wallclock_as_timestamps 1 "
        # "-i rtsp://192.168.144.25:8554/main.264 "
        # "-fflags +genpts -c copy -f "
        # "rtsp -rtsp_transport tcp "
        # "rtsp://34.56.241.85:8554/a8",

        "video_stream_command",
        "ffmpeg -hide_banner -nostdin -loglevel warning "
        "-f v4l2 -i /dev/video0 "
        "-fflags +genpts -c:v libx264 -preset ultrafast -pix_fmt yuv420p "
        "-f rtsp -rtsp_transport tcp "
        "rtsp://34.56.241.85:8554/a8"

        # "video_stream_command",
        # "ffmpeg -rtsp_transport tcp -fflags nobuffer -flags low_delay "
        # "-probesize 32 -analyzeduration 0 "
        # "-use_wallclock_as_timestamps 1 -rtbufsize 0 "
        # "-i rtsp://192.168.144.25:8554/main.264 -c:v libx264 "
        # "-vf scale=720x480 -preset ultrafast -tune zerolatency "
        # "-b:v 500k -r 15  -g 30 -aspect 16:9  -f rtsp -rtsp_transport tcp rtsp://34.56.241.85:8554/a8"

    ).value
    video_use_terminal = bool(
        node.declare_parameter("video_stream_use_terminal", False).value
    )
    video_terminal_program = node.declare_parameter(
        "video_stream_terminal_program", "gnome-terminal"
    ).value

    stream_cfg = PayloadStreamConfig(
        video_value=video_value,
        video_command=video_command,
        video_use_terminal=video_use_terminal,
        video_terminal_program=video_terminal_program,
    )

    return StreamConfig(mode=mode, stream=stream_cfg)


def video_stream(
    *,
    enabled: bool,
    config: PayloadStreamConfig,
    logger: Any,
) -> bool:
    """
    Bật/tắt video stream bằng cách chạy lệnh ffmpeg (hoặc tương đương) trong
    terminal riêng. Trả về True nếu lệnh start đã được kích hoạt thành công,
    False trong các trường hợp còn lại (bao gồm khi yêu cầu tắt stream).
    """

    global _VIDEO_PROCESS

    if enabled:
        if not config.video_command.strip():
            logger.warning("Video stream command rỗng → bỏ qua")
            return False
        if _VIDEO_PROCESS is not None and _VIDEO_PROCESS.poll() is None:
            logger.debug("Video stream đã chạy → không khởi động lại")
            return True

        logger.info("Khởi động video stream bằng videostream_actions")
        try:
            if config.video_use_terminal:
                cmd = [
                    config.video_terminal_program,
                    "--",
                    "bash",
                    "-lc",
                    config.video_command,
                ]
                proc = subprocess.Popen(cmd)
            else:
                proc = subprocess.Popen(
                    config.video_command,
                    shell=True,
                    executable="/bin/bash",
                )
        except FileNotFoundError as exc:
            logger.error(f"Không tìm thấy chương trình video stream: {exc}")
            return False
        except Exception as exc:
            logger.error(f"Lỗi khi khởi động video stream: {exc}")
            return False

        _VIDEO_PROCESS = proc
        return True

    stop_video_stream(logger=logger)
    return False


def stop_video_stream(logger: Any | None = None) -> None:
    """
    Dừng tiến trình stream nếu còn sống. Hàm này chủ yếu phục vụ giai đoạn
    shutdown hoặc khi payload_action bị vô hiệu hóa.
    """

    global _VIDEO_PROCESS

    if _VIDEO_PROCESS is None:
        return

    proc = _VIDEO_PROCESS
    _VIDEO_PROCESS = None

    if proc.poll() is not None:
        if logger:
            logger.info("Video stream đã dừng trước đó")
        return

    if logger:
        logger.info("Dừng video stream")
    try:
        proc.terminate()
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        if logger:
            logger.warning("Video stream không chịu dừng → kill")
        try:
            proc.kill()
        except Exception:
            pass
    except Exception as exc:
        if logger:
            logger.error(f"Lỗi khi dừng video stream: {exc}")


def handle_payload_action(
    *,
    active: bool,
    config: StreamConfig,
    logger: Any,
    publish_video_cb: Optional[Callable[[], None]] = None,
) -> None:
    mode = config.mode
    if mode == "video_stream":
        started = video_stream(enabled=active, config=config.stream, logger=logger)
        if active and started and publish_video_cb:
            publish_video_cb()
        if not active and not started and not publish_video_cb:
            return
        if not active:
            return
    elif mode == "image":
        if active:
            logger.info("Payload mode=image (chưa cài đặt).")
        else:
            logger.debug("Payload image mode dừng.")
    elif mode == "boom_drop":
        if active:
            logger.info("Payload mode=boom_drop (chưa cài đặt).")
        else:
            logger.debug("Payload boom_drop mode dừng.")
    else:
        logger.warning(f"Payload mode '{mode}' chưa được hỗ trợ.")
