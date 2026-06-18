#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

sudo install -m 0755 "$ROOT_DIR"/scripts/*.sh /usr/local/bin/
sudo install -m 0644 "$ROOT_DIR"/systemd/*.service /etc/systemd/system/
if [[ ! -f /etc/default/pharmarobot ]]; then
  sudo install -m 0644 "$ROOT_DIR/systemd/pharmarobot.default" /etc/default/pharmarobot
else
  # Migrate only the legacy default ttyUSB values. Preserve deliberate custom values.
  sudo sed -i \
    -e 's|^FRONT_PORT=/dev/ttyUSB2$|FRONT_PORT=/dev/lidar_front|' \
    -e 's|^BACK_PORT=/dev/ttyUSB1$|BACK_PORT=/dev/lidar_back|' \
    /etc/default/pharmarobot

  if ! grep -q '^ROBOTEQ_PORT=' /etc/default/pharmarobot; then
    echo 'ROBOTEQ_PORT=/dev/roboteq' | sudo tee -a /etc/default/pharmarobot >/dev/null
  fi
fi

sudo systemctl daemon-reload
sudo systemctl enable pharmarobot.service pharma-minimal-nodes.service pharma-lidar-nodes.service

echo "Services installed. Build the image/workspace before starting them."
