#!/usr/bin/env bash
set -euo pipefail
sudo systemctl disable --now robot.service 2>/dev/null || true
sudo rm -f /etc/systemd/system/robot.service
sudo systemctl daemon-reload
echo "robot.service removed"
