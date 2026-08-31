#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "Installing Jetson packages for: $PROJECT_DIR"
sudo apt update
sudo apt install -y \
  git python3-pip python3-venv python3-dev \
  python3-opencv python3-numpy python3-yaml python3-serial \
  v4l-utils gstreamer1.0-tools minicom \
  build-essential cmake unzip

python3 -m venv --system-site-packages "$PROJECT_DIR/.venv"
"$PROJECT_DIR/.venv/bin/python" -m pip install --upgrade pip
"$PROJECT_DIR/.venv/bin/python" -m pip install -r "$PROJECT_DIR/requirements.txt"

sudo usermod -aG dialout,video "$USER"
mkdir -p "$PROJECT_DIR/logs" "$PROJECT_DIR/dataset"

echo
printf '%s\n' "Installation complete." \
  "Log out and log in again so dialout/video group membership applies." \
  "Then run:" \
  "  cd $PROJECT_DIR" \
  "  source .venv/bin/activate" \
  "  python tools/validate_config.py"
