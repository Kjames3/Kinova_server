#!/usr/bin/env bash
# Run once on the Kinova desktop (kinova@10.12.140.145) to install dependencies.
# Usage:  bash install.sh
set -euo pipefail

echo "=== Kinova Robot Server — dependency installer ==="
echo

# ── System packages ────────────────────────────────────────────────────────────
sudo apt-get update -qq
sudo apt-get install -y python3-pip python3-venv libusb-1.0-0-dev usbutils

# ── Python virtual environment ─────────────────────────────────────────────────
python3 -m venv venv
# shellcheck source=/dev/null
source venv/bin/activate
pip install --upgrade pip

# ── FastAPI / uvicorn / OpenCV / NumPy ────────────────────────────────────────
pip install "fastapi>=0.110.0" "uvicorn[standard]>=0.29.0" \
            "numpy>=1.24.0" "opencv-python>=4.9.0"

# ── Intel RealSense SDK ────────────────────────────────────────────────────────
pip install pyrealsense2

# ── Luxonis DepthAI (OAK-D) ───────────────────────────────────────────────────
pip install depthai
# Set up udev rules so the camera is accessible without sudo
echo "Installing OAK-D udev rules…"
curl -fL https://docs.luxonis.com/install_dependencies.sh | bash

# ── Kinova kortex_api ──────────────────────────────────────────────────────────
# Download the pre-built wheel from Kinova's public Artifactory.
# If a newer version is available visit:
#   https://github.com/Kinovarobotics/kortex/blob/master/api_python/README.md
KORTEX_VER="2.6.0"
KORTEX_WHL="kortex_api-${KORTEX_VER}.post3-py3-none-any.whl"
KORTEX_URL="https://artifactory.kinovaapps.com/artifactory/generic-public/kortex/API/${KORTEX_VER}/${KORTEX_WHL}"

echo "Downloading kortex_api ${KORTEX_VER}…"
wget -q "${KORTEX_URL}" -O "/tmp/${KORTEX_WHL}" || {
    echo ""
    echo "  Direct download failed. Install manually:"
    echo "    1. Download the wheel from Kinova's Artifactory or GitHub Releases"
    echo "    2. Run: pip install /path/to/${KORTEX_WHL}"
    echo ""
}
[ -f "/tmp/${KORTEX_WHL}" ] && pip install "/tmp/${KORTEX_WHL}"

# ── Firewall: open port 8080 ───────────────────────────────────────────────────
if command -v ufw &>/dev/null; then
    sudo ufw allow 8080/tcp
    echo "ufw: port 8080 opened"
fi

echo
echo "=== Installation complete ==="
echo "Activate the venv and start the server:"
echo "  source venv/bin/activate"
echo "  python server.py"
echo
echo "Then open:  http://10.12.140.145:8080"
