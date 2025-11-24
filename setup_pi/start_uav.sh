#!/bin/bash

# --- Cấu hình ---
# User sẽ chạy các node ROS 2
USERNAME="ctuav3"

# Phiên bản ROS 2
ROS_DISTRO="jazzy"
# --- Hết cấu hình ---

WORKSPACE_DIR="/home/$USERNAME/giam_sat_uav"

echo "--- Bắt đầu dịch vụ UAV (Chế độ phân quyền Root/User) ---"

# ==========================================================
# == PHẦN 1: CÁC LỆNH CHẠY VỚI QUYỀN ROOT ==
# (Script này được systemd chạy với quyền root)
# ==========================================================

# 1. Chạy microRTPS agent (bằng root)
echo "Starting RTPS Agent (as root)..."
MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600 &
RTPS_PID=$!

sleep 3 # Đợi agent khởi động

# 2. Cấu hình camera A8mini (bằng root)
echo "Configuring eth0 (as root)..."
/usr/sbin/ethtool -s eth0 speed 10 duplex full autoneg off
sleep 1

# ==========================================================
# == PHẦN 2: CÁC LỆNH CHẠY VỚI QUYỀN USER (ctuav3) ==
# ==========================================================

# Chuẩn bị chuỗi lệnh source môi trường cho user
USER_SETUP_CMD="source /opt/ros/$ROS_DISTRO/setup.bash; source $WORKSPACE_DIR/install/setup.bash"
LOG_DIR="/home/$USERNAME"
# Ensure log dir exists (in case /home is mounted late) and prepare file before redirect
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/uav_log_$(date +%Y%m%d_%H%M%S).txt"
# Create the file and set permissions so user processes can write if needed
touch "$LOG_FILE"
chown $USERNAME:$USERNAME "$LOG_FILE" || true
chmod 664 "$LOG_FILE" || true

# Prepend ISO8601-UTC (micro-giây thật) cho MỌI dòng log và ghi vào file + console
exec > >(
  while IFS= read -r line; do
    printf '%s %s\n' "$(date -u +'%Y-%m-%dT%H:%M:%S.%6NZ')" "$line"
  done | tee -a "$LOG_FILE"
) 2>&1
echo "=== UAV startup log ==="
echo "Time: $(date)"
echo "----------------------"

# 3. Chạy node MQTT bridge (bằng user ctuav3)
echo "Starting MQTT Bridge (as $USERNAME)..."
MQTT_CMD="ros2 run uav_mqtt_bridge gimbal"

# Chạy lệnh với quyền của $USERNAME
# -H: Đặt biến $HOME thành thư mục nhà của user
# -u $USERNAME: Chỉ định chạy với user
# bash -c "...": Chạy một shell mới để source và thực thi lệnh
sudo -H -u $USERNAME bash -c "$USER_SETUP_CMD; $MQTT_CMD" &

# 4. Chạy node GPS Setpoint (bằng user ctuav3)
echo "Starting GPS Setpoint (as $USERNAME)..."
GPS_CMD="ros2 run uav_gps_setpoint uav_gps_setpoint"
sudo -H -u $USERNAME bash -c "$USER_SETUP_CMD; $GPS_CMD" &


# Hàm cleanup khi service bị stop
cleanup() {
    echo "Shutting down nodes..."
    kill $RTPS_PID # Giết tiến trình chạy bằng root
    
    # Giết các tiến trình chạy bằng user
    # Dùng pkill để tìm đúng tiến trình con
    sudo -u $USERNAME pkill -f 'uav_mqtt_bridge'
    sudo -u $USERNAME pkill -f 'uav_gps_setpoint'
    
    exit 0
}

trap cleanup SIGINT SIGTERM

echo "--- Tất cả các node đã chạy. Service đang hoạt động. ---"

# Đợi tiến trình root (RTPS Agent) kết thúc
# Nếu agent chết, service sẽ tự động restart (theo cấu hình .service)
wait $RTPS_PID

