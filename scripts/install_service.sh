#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
USER_NAME="${SUDO_USER:-$USER}"
TEMPLATE="$PROJECT_DIR/systemd/robot.service.in"
TEMP_FILE="$(mktemp)"
sed -e "s|@USER@|$USER_NAME|g" -e "s|@PROJECT_DIR@|$PROJECT_DIR|g" "$TEMPLATE" > "$TEMP_FILE"
sudo install -m 0644 "$TEMP_FILE" /etc/systemd/system/robot.service
rm -f "$TEMP_FILE"
sudo systemctl daemon-reload
sudo systemctl enable robot.service
sudo systemctl restart robot.service
sudo systemctl --no-pager --full status robot.service || true
echo "Logs: journalctl -u robot.service -f"
