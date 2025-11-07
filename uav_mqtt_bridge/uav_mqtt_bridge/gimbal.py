#!/usr/bin/env python3
import json, threading, ssl, time
from datetime import datetime, timezone, timedelta
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from px4_msgs.msg import VehicleGlobalPosition, VehicleStatus, VehicleAttitude, BatteryStatus
from uav_msgs.msg import TargetGlobal, UserCmd, EventUav
import paho.mqtt.client as mqtt

try:
    from .siyi_sdk import SIYISDK  # type: ignore
except ImportError:  # pragma: no cover
    from siyi_sdk import SIYISDK  # type: ignore

# ---- QoS (BEST_EFFORT, depth=1) ----
qos_px4 = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

# ---- nav_state -> mode string ----
NAV_STATE_NAME = {

    # Các chế độ điều khiển bằng tay (Manual)
    VehicleStatus.NAVIGATION_STATE_MANUAL:        "MANUAL",
    VehicleStatus.NAVIGATION_STATE_STAB:          "STABILIZE",
    VehicleStatus.NAVIGATION_STATE_ALTCTL:        "ALTCTL",
    VehicleStatus.NAVIGATION_STATE_POSCTL:        "POSITION",

    # Các chế độ tự động (Autonomous)
    VehicleStatus.NAVIGATION_STATE_AUTO_MISSION:  "MISSION",
    VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:   "HOLD", # Trạng thái này cũng được dùng cho Loiter
    VehicleStatus.NAVIGATION_STATE_AUTO_RTL:      "RETURN_TO_HOME",
    VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF:  "TAKEOFF",
    VehicleStatus.NAVIGATION_STATE_AUTO_LAND:     "LANDING",

    # Chế độ điều khiển từ bên ngoài
    VehicleStatus.NAVIGATION_STATE_OFFBOARD:      "OFFBOARD",
}

# (Ngay sau dictionary NAV_STATE_NAME)

AUTONOMOUS_STATES = {
    VehicleStatus.NAVIGATION_STATE_AUTO_MISSION,
    VehicleStatus.NAVIGATION_STATE_AUTO_LOITER,
    VehicleStatus.NAVIGATION_STATE_AUTO_RTL,
    VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF,
    VehicleStatus.NAVIGATION_STATE_AUTO_LAND,
    VehicleStatus.NAVIGATION_STATE_OFFBOARD,  # Offboard cũng được coi là tự động
}

MANUAL_STATES = {
    VehicleStatus.NAVIGATION_STATE_MANUAL,
    VehicleStatus.NAVIGATION_STATE_STAB,
    VehicleStatus.NAVIGATION_STATE_ALTCTL,
    VehicleStatus.NAVIGATION_STATE_POSCTL,
}

def normalize_mode(nav_state: int) -> str:
    return NAV_STATE_NAME.get(nav_state, f"UNKNOWN({nav_state})")

def norm_lat(lat):
    v = float(lat)
    if abs(v) > 1800:  # trường hợp dạng int*1e7
        v /= 1e7
    return v

def norm_lon(lon):
    v = float(lon)
    if abs(v) > 1800:
        v /= 1e7
    return v

def norm_alt(alt):
    v = float(alt)
    if abs(v) > 100000:  # nếu có thể là mm
        v /= 1e3
    return v

def battery_percent(remain_field) -> float | None:
    """
    PX4 BatteryStatus.remaining đôi khi [0..1], đôi khi đã là %.
    Trả về % (0..100). Nếu None/NaN -> None.
    """
    if remain_field is None:
        return None
    try:
        p = float(remain_field)
    except Exception:
        return None
    if math.isnan(p):
        return None
    if 0.0 <= p <= 1.0:
        p *= 100.0
    return max(0.0, min(100.0, p))

def yaw_from_quat_deg(q):
    """
    PX4 VehicleAttitude.q = [w, x, y, z] (scalar-first).
    Trả về yaw (độ, 0..360).
    """
    if len(q) != 4:
        return None
    w, x, y, z = [float(v) for v in q]
    # yaw (Z) từ quaternion
    # Reference: yaw = atan2(2(wz + xy), 1 - 2(y^2 + z^2))
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw_rad = math.atan2(siny_cosp, cosy_cosp)
    yaw_deg = math.degrees(yaw_rad)
    # chuẩn hóa 0..360
    if yaw_deg < 0:
        yaw_deg += 360.0
    return yaw_deg

class Px4MqttHeartbeat(Node):
    def __init__(self):
        super().__init__("px4_mqtt_heartbeat")

        # ---- Params ----
        self.vehicle_id   = self.declare_parameter("vehicle_id", "uav3").value
        self.mqtt_host    = self.declare_parameter(
            "mqtt_host", "h1171119.ala.dedicated.aws.emqxcloud.com"
        ).value
        self.mqtt_port    = self.declare_parameter("mqtt_port", 8883).value
        self.mqtt_user    = self.declare_parameter("mqtt_user", "nhutctuav").value
        self.mqtt_pass    = self.declare_parameter("mqtt_pass", "123456Aa").value
        self.mqtt_use_tls = self.declare_parameter("mqtt_use_tls", True).value
        self.command_topic = self.declare_parameter(
            "command_topic", f"{self.vehicle_id}/cmd"
        ).value
        self.target_topic = self.declare_parameter(
            "target_global_topic", "/target_global"
        ).value
        self.user_cmd_topic = self.declare_parameter(
            "user_cmd_topic", "/user_cmd"
        ).value
        self.event_topic = self.declare_parameter(
            "event_topic", f"{self.vehicle_id}/event"
        ).value


        self.gimbal_enable = bool(self.declare_parameter("gimbal_enable", True).value)
        self.gimbal_host = self.declare_parameter("gimbal_host", "192.168.144.25").value
        self.gimbal_port = int(self.declare_parameter("gimbal_port", 37260).value)
        self.gimbal_debug = bool(self.declare_parameter("gimbal_debug", False).value)


        self.event_sub_topic = self.declare_parameter(
            "event_sub_topic", "/event_uav"
        ).value
        self.telemetry_topic = self.declare_parameter(
            "topic", f"{self.vehicle_id}/telemetry"
        ).value
        self.rate_hz = float(self.declare_parameter("rate_hz", 5.0).value)
        self.heartbeat_topic = self.declare_parameter(
            "heartbeat_topic", f"{self.vehicle_id}/heartbeat"
        ).value
        self.heartbeat_rate_hz = float(
            self.declare_parameter("heartbeat_rate_hz", 2.0).value
        )

        # ---- MQTT (only publish) ----
        self.mqtt = mqtt.Client(client_id=f"{self.vehicle_id}-hb", clean_session=True)
        if self.mqtt_user:
            self.mqtt.username_pw_set(self.mqtt_user, self.mqtt_pass)
        if self.mqtt_use_tls:
            self.mqtt.tls_set(cert_reqs=ssl.CERT_NONE)
            self.mqtt.tls_insecure_set(True)
        self.mqtt.connect(self.mqtt_host, self.mqtt_port, keepalive=60)
        self.mqtt.on_message = self.on_mqtt_message
        if self.command_topic:
            self.mqtt.subscribe(self.command_topic)
        self.mqtt.loop_start()

        # ---- ROS2 subs ----
        self.last_gps: VehicleGlobalPosition | None = None
        self.last_status: VehicleStatus | None = None
        self.last_att: VehicleAttitude | None = None
        self.last_batt_pct: float | None = None
        self.time_remaining_sec: float | None = None
        self.command_lock = threading.Lock()
        self.last_command: dict | None = None
        self._gimbal_lock = threading.Lock() ############################## gimbal
        self.target_pub = self.create_publisher(TargetGlobal, self.target_topic, 10)
        self.user_cmd_pub = self.create_publisher(UserCmd, self.user_cmd_topic, 10)
        self.create_subscription(EventUav, self.event_sub_topic, self.cb_event, 10)

        self._mqtt_disconnect_time: float | None = None
        self._mqtt_rtl_sent = False
        self._mqtt_watchdog_timer = self.create_timer(1.0, self._mqtt_watchdog_timer_cb)


        self.create_subscription(VehicleGlobalPosition, "/fmu/out/vehicle_global_position",
                                 self.cb_gps, qos_px4)
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v1",
                                 self.cb_status, qos_px4)
        self.create_subscription(VehicleAttitude, "/fmu/out/vehicle_attitude",
                                 self.cb_att, qos_px4)
        self.create_subscription(BatteryStatus, "/fmu/out/battery_status_v1",
                                 self.cb_batt, qos_px4)

        # ---- Timer publish ----
        telemetry_period = 1.0 / self.rate_hz if self.rate_hz > 0 else 0.2
        self.telemetry_timer = self.create_timer(telemetry_period, self.publish_telemetry)

        heartbeat_period = (
            1.0 / self.heartbeat_rate_hz if self.heartbeat_rate_hz > 0 else 0.5
        )
        self.heartbeat_timer = self.create_timer(heartbeat_period, self.publish_heartbeat)

        self.get_logger().info(
            f"Telemetry → MQTT {self.mqtt_host}:{self.mqtt_port} topic='{self.telemetry_topic}' "
            f"(vehicle_id={self.vehicle_id}, hz={self.rate_hz}, tls={self.mqtt_use_tls})"
        )
        if self.heartbeat_topic:
            self.get_logger().info(
                f"Heartbeat → MQTT {self.mqtt_host}:{self.mqtt_port} topic='{self.heartbeat_topic}' "
                f"(vehicle_id={self.vehicle_id}, hz={self.heartbeat_rate_hz}, tls={self.mqtt_use_tls})"
            )


        self.mqtt.on_connect = self.on_mqtt_connect
        self.mqtt.on_disconnect = self.on_mqtt_disconnect
        self.mqtt.on_message = self.on_mqtt_message

    # ---------- Callbacks ----------
    def cb_gps(self, msg: VehicleGlobalPosition):
        self.last_gps = msg

    def cb_status(self, msg: VehicleStatus):
        self.last_status = msg

    def cb_att(self, msg: VehicleAttitude):
        self.last_att = msg

    def cb_batt(self, msg: BatteryStatus):
        self.last_batt_pct = battery_percent(msg.remaining)
        self.time_remaining_sec = float(msg.time_remaining_s)

    def cb_event(self, msg: EventUav):
        self.publish_event("target reach", msg.target_reach)

    # ---------- MQTT commands ----------
    def on_mqtt_message(self, client, userdata, msg):
        try:
            raw = msg.payload.decode("utf-8")
        except UnicodeDecodeError as exc:
            self.get_logger().warning(
                f"MQTT command decode error on topic '{msg.topic}': {exc}"
            )
            return
        try:
            data = json.loads(raw)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(
                f"MQTT command JSON error on topic '{msg.topic}': {exc}"
            )
            return
        else:
            self.get_logger().info(
                f"MQTT command raw payload on topic '{msg.topic}': {raw}"
            )

        expected_keys = (
            "connect",
            "set_home",
            "arm",
            "pause",
            "continue",
            "rtl",
            "mission",
            "waypoint",
            "flag",
        )
        missing = [key for key in expected_keys if key not in data]
        if missing:
            self.get_logger().warning(
                f"MQTT command missing keys {missing} on topic '{msg.topic}'"
            )
            return

        waypoint = data["waypoint"]
        if not isinstance(waypoint, list):
            self.get_logger().warning(
                f"MQTT command has non-list waypoint on topic '{msg.topic}'"
            )
            return

        raw_gimbal_flag = data.get("gimbal", False)
        if isinstance(raw_gimbal_flag, bool):
            gimbal_cmd = raw_gimbal_flag
        elif isinstance(raw_gimbal_flag, str):
            gimbal_cmd = raw_gimbal_flag.strip().lower() in ("1", "true", "yes", "on")
        else:
            gimbal_cmd = bool(raw_gimbal_flag)

        raw_angles = data.get("angle")
        if raw_angles is None:
            raw_angles = data.get("angles")
        if raw_angles is None:
            raw_angles = data.get("gimbal_angle")
        if raw_angles is None:
            raw_angles = data.get("gimbal_angles")

        gimbal_pitch: Optional[float] = None
        gimbal_yaw: Optional[float] = None
        if isinstance(raw_angles, (list, tuple)):
            try:
                gimbal_pitch = float(raw_angles[0])
            except (IndexError, TypeError, ValueError):
                gimbal_pitch = None
            try:
                gimbal_yaw = float(raw_angles[1])
            except (IndexError, TypeError, ValueError):
                gimbal_yaw = None
        elif isinstance(raw_angles, dict):
            try:
                if "pitch" in raw_angles:
                    gimbal_pitch = float(raw_angles["pitch"])
                elif "x" in raw_angles:
                    gimbal_pitch = float(raw_angles["x"])
            except (TypeError, ValueError):
                gimbal_pitch = None
            try:
                if "yaw" in raw_angles:
                    gimbal_yaw = float(raw_angles["yaw"])
                elif "y" in raw_angles:
                    gimbal_yaw = float(raw_angles["y"])
            except (TypeError, ValueError):
                gimbal_yaw = None
        elif isinstance(raw_angles, str):
            tokens = [item for item in raw_angles.replace(",", " ").split() if item]
            numeric_vals = []
            for token in tokens:
                try:
                    numeric_vals.append(float(token))
                    continue
                except ValueError:
                    pass
                if "=" in token:
                    _, value = token.split("=", 1)
                elif ":" in token:
                    _, value = token.split(":", 1)
                else:
                    continue
                try:
                    numeric_vals.append(float(value))
                except ValueError:
                    continue

            if len(numeric_vals) >= 1:
                gimbal_pitch = numeric_vals[0]
            if len(numeric_vals) >= 2:
                gimbal_yaw = numeric_vals[1]

        cmd = {
            "connect": bool(data["connect"]),
            "set_home": bool(data["set_home"]),
            "arm": bool(data["arm"]),
            "pause": bool(data["pause"]),
            "resume": bool(data["continue"]),
            "rtl": bool(data["rtl"]),
            "mission": bool(data["mission"]),
            "waypoint": waypoint,
            "flag": str(data["flag"]),
            "gimbal": gimbal_cmd,
            "gimbal_pitch": gimbal_pitch,
            "gimbal_yaw": gimbal_yaw,
        }

        with self.command_lock:
            self.last_command = cmd

        self.publish_response(cmd)

        if cmd["mission"]:
            self.publish_target_global(waypoint)

        if cmd["arm"] or cmd["pause"] or cmd["resume"] or cmd["rtl"] or cmd["set_home"]:
            self.publish_user_cmd(cmd)

        if cmd["gimbal"]:
            self.handle_gimbal_command(cmd["gimbal_pitch"], cmd["gimbal_yaw"])

        self.get_logger().info(
            f"Received command from MQTT '{msg.topic}': "
            f"connect={cmd['connect']} set_home={cmd['set_home']} arm={cmd['arm']} "
            f"pause={cmd['pause']} resume={cmd['resume']} rtl={cmd['rtl']} "
            f"mission={cmd['mission']} flag={cmd['flag']} "
            f"gimbal={cmd['gimbal']} pitch={cmd['gimbal_pitch']} yaw={cmd['gimbal_yaw']}"
        )

#loss connect cloud check
    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info(
                f"MQTT connected to {self.mqtt_host}:{self.mqtt_port}"
            )
            if self.command_topic:
                client.subscribe(self.command_topic)
            self._mqtt_disconnect_time = None
            self._mqtt_rtl_sent = False
        else:
            self.get_logger().error(
                f"MQTT failed to connect to {self.mqtt_host}:{self.mqtt_port} rc={rc}"
            )

    def on_mqtt_disconnect(self, client, userdata, rc):
        if rc == 0:
            self.get_logger().info("MQTT connection closed cleanly")
            return

        try:
            err_text = mqtt.error_string(rc)
        except Exception:
            err_text = "unknown reason"

        self.get_logger().error(
            f"Lost connection to MQTT broker {self.mqtt_host}:{self.mqtt_port} "
            f"rc={rc} ({err_text})"
        )
        if self._mqtt_disconnect_time is None:
            self._mqtt_disconnect_time = time.time()
            self._mqtt_rtl_sent = False

    def _mqtt_watchdog_timer_cb(self):
        if self._mqtt_disconnect_time is None:
            return
        if self._mqtt_rtl_sent:
            return
        if (time.time() - self._mqtt_disconnect_time) < 60.0:
            return

        self._mqtt_rtl_sent = True
        self.get_logger().error(
            "MQTT connection unavailable for 60s → issuing RTL command"
        )
        try:
            self.publish_user_cmd({"rtl": True})
        except Exception as exc:
            self.get_logger().error(
                f"Failed to publish RTL command after MQTT loss: {exc}"
            )
##
    def publish_target_global(self, waypoint_list):
        if not waypoint_list:
            self.get_logger().warning(
                f"Mission command has empty waypoint list; skip publish to '{self.target_topic}'"
            )
            return

        latitudes = []
        longitudes = []
        altitudes = []
        for idx, entry in enumerate(waypoint_list):
            if isinstance(entry, dict):
                lat = entry.get("lat", entry.get("latitude"))
                lon = entry.get("lon", entry.get("longitude"))
                alt = entry.get("alt", entry.get("alt_m"))
            elif isinstance(entry, (list, tuple)) and len(entry) >= 3:
                lat, lon, alt = entry[0], entry[1], entry[2]
            else:
                self.get_logger().warning(
                    f"Ignoring waypoint[{idx}] with unsupported format: {entry}"
                )
                continue
            try:
                latitudes.append(float(lat))
                longitudes.append(float(lon))
                altitudes.append(float(alt))
            except (TypeError, ValueError):
                self.get_logger().warning(
                    f"Ignoring waypoint[{idx}] with non-numeric values: {entry}"
                )

        if not latitudes:
            self.get_logger().warning(
                f"No valid waypoint after parsing; skip publish to '{self.target_topic}'"
            )
            return

        msg = TargetGlobal()
        msg.lat = latitudes
        msg.lon = longitudes
        msg.alt_m = altitudes
        self.target_pub.publish(msg)
        self.get_logger().info(
            f"Published {len(latitudes)} waypoint(s) to '{self.target_topic}'"
        )
# fgs
    def publish_user_cmd(self, cmd: dict):
        
        msg = UserCmd()
        msg.arm = bool(cmd.get("arm"))
        msg.rth = bool(cmd.get("rtl"))
        msg.pause = bool(cmd.get("pause"))
        msg.resume = bool(cmd.get("resume"))
        msg.set_home = bool(cmd.get("set_home"))

        self.user_cmd_pub.publish(msg)
        self.get_logger().info(
            f"Published UserCmd arm={msg.arm} rth={msg.rth} pause={msg.pause} "
            f"resume={msg.resume} set_home={msg.set_home} to '{self.user_cmd_topic}'"
        )

    def handle_gimbal_command(self, pitch: Optional[float], yaw: Optional[float]):
        if not self.gimbal_enable:
            self.get_logger().warning("Gimbal command received but gimbal_enable=False → ignoring")
            return

        def worker():
            with self._gimbal_lock:
                self._execute_gimbal_command(pitch, yaw)

        threading.Thread(target=worker, daemon=True).start()

    def _execute_gimbal_command(self, pitch: Optional[float], yaw: Optional[float]):
        try:
            cam = SIYISDK(server_ip=self.gimbal_host, port=self.gimbal_port, debug=self.gimbal_debug)
        except Exception as exc:
            self.get_logger().error(f"Failed to create SIYI SDK instance: {exc}")
            return

        try:
            if not cam.connect():
                self.get_logger().error(
                    f"Unable to connect to SIYI gimbal at {self.gimbal_host}:{self.gimbal_port}"
                )
                return

            if pitch is None and yaw is None:
                self.get_logger().info("Executing gimbal center command")
                cam.requestCenterGimbal()
            else:
                yaw_cmd = 0.0 if yaw is None else float(yaw)
                pitch_cmd = 0.0 if pitch is None else float(pitch)
                self.get_logger().info(
                    f"Executing gimbal angle command pitch={pitch_cmd} yaw={yaw_cmd}"
                )
                cam.setGimbalRotation(yaw_cmd, pitch_cmd)
        except Exception as exc:
            self.get_logger().error(f"Error while executing gimbal command: {exc}")
        finally:
            try:
                cam.disconnect()
            except Exception:
                pass

    def _current_timestamp(seff) -> str:
        """Lấy thời gian hiện tại theo đồng hồ của máy tính (local system time)."""
        return datetime.now().isoformat()

    def _build_location(self):
        location = {
            "latitude": None,
            "longitude": None,
            "altitude": None,
        }

        if self.last_gps is not None:
            location["latitude"] = norm_lat(self.last_gps.lat)
            location["longitude"] = norm_lon(self.last_gps.lon)
            location["altitude"] = norm_alt(self.last_gps.alt)

        return location

    def _build_state_snapshot(self) -> Optional[dict]:
        if self.last_status is None:
            return None

        st = self.last_status
        armed = st.arming_state == VehicleStatus.ARMING_STATE_ARMED
        if not armed:
            state_str = "DISARMED"
        elif st.nav_state in AUTONOMOUS_STATES:
            state_str = "AUTONOMOUS"
        elif st.nav_state in MANUAL_STATES:
            state_str = "MANUAL"
        else:
            state_str = "ARMED"

        heading_deg = None
        if self.last_att is not None:
            heading_deg = yaw_from_quat_deg(self.last_att.q)

        gps = self.last_gps
        gps_block = {
            "latitude": norm_lat(gps.lat) if gps is not None else None,
            "longitude": norm_lon(gps.lon) if gps is not None else None,
            "altitude": norm_alt(gps.alt) if gps is not None else None,
            "heading": None if heading_deg is None else round(heading_deg, 2),
        }

        return {
            "state": state_str,
            "mode": normalize_mode(int(st.nav_state)),
            "battery": None if self.last_batt_pct is None else round(self.last_batt_pct, 1),
            "time_remaining_sec": None if self.time_remaining_sec is None else round(self.time_remaining_sec, 1),
            "gps": gps_block,
        }

    def publish_event(self, event_type: str, value):
        if not self.event_topic or not value:
            return

        location = self._build_location()

        if event_type == "target reach":
            value_str = f"target number {int(value)} reached"
        else:
            value_str = f"{event_type}: {int(value)}"

        payload = {
            "id": self.vehicle_id,
            "type": "drone_event",
            "value": value_str,
            "flag": "", #event co flag, response k co flag
            "location": location,
            "timestamp": self._current_timestamp(),
        }

        try:
            self.mqtt.publish(self.event_topic, json.dumps(payload), qos=1)
        except Exception as exc:
            self.get_logger().error(
                f"MQTT event publish error to '{self.event_topic}': {exc}"
            )
        else:
            self.get_logger().info(
                f"Sent event '{event_type}' value={value} to '{self.event_topic}'"
            )

    def publish_response(self, cmd: dict):
        if not self.event_topic:
            return

        action_order = (
            "arm",
            "pause",
            "resume",
            "rtl",
            "mission",
            "connect",
            "set_home",
        )
        response_action = "none"
        for key in action_order:
            if cmd.get(key):
                response_action = key
                break

        payload = {
            "id": self.vehicle_id,
            "type": "response",
            "value": f"{self.vehicle_id} received command {cmd.get('flag', '')}",
            "flag": cmd.get("flag", ""),
            "location": self._build_location(),
            "timestamp": self._current_timestamp(),
        }

        try:
            self.mqtt.publish(self.event_topic, json.dumps(payload), qos=1)
        except Exception as exc:
            self.get_logger().error(
                f"MQTT response publish error to '{self.event_topic}': {exc}"
            )
        else:
            self.get_logger().info(
                f"Sent MQTT response '{response_action}' to '{self.event_topic}'"
            )

    # ---------- Publish ----------
    def publish_telemetry(self):
        if not self.telemetry_topic:
            return

        state_snapshot = self._build_state_snapshot()
        if state_snapshot is None:
            return

        # Telemetry payload mirrors legacy heartbeat content.
        payload = {
            "uav_id": self.vehicle_id,
            "timestamp": datetime.now().astimezone().isoformat(),
            "telemetry": state_snapshot,
        }

        try:
            self.mqtt.publish(self.telemetry_topic, json.dumps(payload), qos=1)
        except Exception as e:
            self.get_logger().error(f"MQTT telemetry publish error: {e}")

    def publish_heartbeat(self):
        if not self.heartbeat_topic:
            return

        payload = {
            "uav_id": self.vehicle_id,
            "timestamp": datetime.now().astimezone().isoformat(),
        }

        try:
            self.mqtt.publish(self.heartbeat_topic, json.dumps(payload), qos=0)
        except Exception as exc:
            self.get_logger().error(f"MQTT heartbeat publish error: {exc}")

    # ---------- Shutdown ----------
    def destroy_node(self):
        try:
            self.mqtt.loop_stop()
            self.mqtt.disconnect()
        except Exception:
            pass
        return super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = Px4MqttHeartbeat()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
