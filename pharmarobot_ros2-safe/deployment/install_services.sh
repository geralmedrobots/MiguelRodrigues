#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$ROOT_DIR/.." && pwd)"
LEGACY_WS_DIR="/home/medrobots/pharmarobot/pharmarobot/pharmarobot_ros2-master"
LEGACY_PHARMA_IMAGE="pharmarobot:2026-06-12-l1"
DEFAULT_PHARMA_IMAGE="pharmarobot:clean"

sudo install -m 0755 "$ROOT_DIR"/scripts/*.sh /usr/local/bin/
sudo install -m 0644 "$ROOT_DIR"/systemd/*.service /etc/systemd/system/
if [[ ! -f /etc/default/pharmarobot ]]; then
  sudo install -m 0644 "$ROOT_DIR/systemd/pharmarobot.default" /etc/default/pharmarobot
else
  # Migrate only the legacy default ttyUSB values. Preserve deliberate custom values.
  sudo sed -i \
    -e 's|^FRONT_PORT=/dev/ttyUSB2$|FRONT_PORT=/dev/lidar_front|' \
    -e 's|^BACK_PORT=/dev/ttyUSB1$|BACK_PORT=/dev/lidar_back|' \
    -e "s|^PHARMA_WS_DIR=${LEGACY_WS_DIR}$|PHARMA_WS_DIR=${REPO_ROOT}|" \
    -e "s|^PHARMA_IMAGE=${LEGACY_PHARMA_IMAGE}$|PHARMA_IMAGE=${DEFAULT_PHARMA_IMAGE}|" \
    /etc/default/pharmarobot

  if ! grep -q '^PHARMA_WS_DIR=' /etc/default/pharmarobot; then
    printf 'PHARMA_WS_DIR=%s\n' "$REPO_ROOT" |
      sudo tee -a /etc/default/pharmarobot >/dev/null
  fi

  if ! grep -q '^ROBOTEQ_PORT=' /etc/default/pharmarobot; then
    echo 'ROBOTEQ_PORT=/dev/roboteq' | sudo tee -a /etc/default/pharmarobot >/dev/null
  fi

  while IFS= read -r setting; do
    name="${setting%%=*}"
    if ! grep -q "^${name}=" /etc/default/pharmarobot; then
      printf '%s\n' "$setting" | sudo tee -a /etc/default/pharmarobot >/dev/null
    fi
  done <<'EOF'
ROS_LOCALHOST_ONLY=0
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
TELEDEX_REFERENCE_ENABLED=1
TELEDEX_REFERENCE_HOST_DIR=/home/medrobots/teledex_reference
TELEDEX_REFERENCE_CONTAINER_DIR=/opt/teledex_reference
D455_SENSOR_CONTAINER=pharmarobot_d455_sensor
D455_SENSOR_IMAGE=pharmarobot:d455-sensor
D455_SERIAL_NUMBER=146222250608
D455_USB_SERIAL_NUMBER=151223061922
D455_SENSOR_EVIDENCE_ROOT=/var/log/pharmarobot/d455-sensor
D455_SENSOR_IMAGE_MANIFEST=/var/lib/pharmarobot/d455-sensor-image.env
D455_SENSOR_OWNERSHIP_RECORD=/var/lib/pharmarobot/d455-sensor-container.json
EOF

  # Retire superseded main-container D455 settings without deleting a local
  # administrator's file. They are ignored by the new production lifecycle.
  sudo sed -i \
    -e 's/^D455_IMU_ENABLED=/# deprecated: D455_IMU_ENABLED=/' \
    -e 's/^D455_IMU_REQUIRED=/# deprecated: D455_IMU_REQUIRED=/' \
    -e 's/^D455_APPARMOR_PROFILE=/# deprecated: D455_APPARMOR_PROFILE=/' \
    /etc/default/pharmarobot
fi

sudo systemctl daemon-reload
sudo systemctl enable \
  pharmarobot.service \
  pharma-minimal-nodes.service \
  pharma-lidar-nodes.service \
  pharma-d455-imu.service

echo "Services installed. Build the image/workspace before starting them."
