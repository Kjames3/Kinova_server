#!/usr/bin/env bash
# Run once on the Kinova desktop (kinova@10.12.140.145) to install dependencies.
# Usage:  bash install.sh
#
# ROS2 note (PLAN 3.0): rclpy must come from the ROS install, NOT pip. We create
# the venv with --system-site-packages so the ROS2 Python packages stay visible,
# and you must `source /opt/ros/humble/setup.bash` before starting the server.
set -euo pipefail

echo "=== Kinova Robot Server — dependency installer ==="
echo

# ── System packages ────────────────────────────────────────────────────────────
sudo apt-get update -qq
sudo apt-get install -y python3-pip python3-venv libusb-1.0-0-dev usbutils

# ── ROS2 Humble (Ubuntu 22.04) ─────────────────────────────────────────────────
# Skip silently if ROS2 isn't set up yet (steps 6-10 of the plan need it).
if [ -d /opt/ros/humble ]; then
    echo "ROS2 Humble found — installing bridge + driver packages…"
    sudo apt-get install -y \
        ros-humble-ros-base ros-humble-cv-bridge ros-humble-image-transport \
        ros-humble-realsense2-camera ros-humble-depthai-ros || true
    # Optional standard bridges (PLAN 3.2):
    sudo apt-get install -y ros-humble-foxglove-bridge ros-humble-rosbridge-suite || true
else
    echo "ROS2 Humble not found at /opt/ros/humble — skipping ROS2 packages."
    echo "  (Server still runs; the ROS2 panel reports BRIDGE OFFLINE until installed.)"
fi

# ── Python virtual environment (sees ROS2 python via system site-packages) ─────
python3 -m venv --system-site-packages venv
# shellcheck source=/dev/null
source venv/bin/activate
pip install --upgrade pip

# ── FastAPI / uvicorn / OpenCV / NumPy ────────────────────────────────────────
pip install "fastapi>=0.110.0" "uvicorn[standard]>=0.29.0" \
            "numpy>=1.24.0" "opencv-python>=4.9.0" "psutil>=5.9.0"

# ── Intel RealSense SDK ────────────────────────────────────────────────────────
pip install pyrealsense2

# ── Luxonis DepthAI (OAK-D) ───────────────────────────────────────────────────
pip install depthai
echo "Installing OAK-D udev rules…"
curl -fL https://docs.luxonis.com/install_dependencies.sh | bash

# ── Kinova kortex_api ──────────────────────────────────────────────────────────
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
echo "Set a password and start the server (source ROS2 first so rclpy is found):"
echo "  source /opt/ros/humble/setup.bash"
echo "  source ~/workspace/ros2_kortex_ws/install/setup.bash   # for the ROS2 panel"
echo "  source venv/bin/activate"
echo "  export SERVER_PASSWORD='choose-a-password'"
echo "  python server.py"
echo
echo "To enable the container-insertion action (PLAN 3.0.1), build the workspace"
echo "interface, then re-source the overlay above:"
echo "  cd ~/workspace/ros2_kortex_ws"
echo "  colcon build --executor sequential --symlink-install \\"
echo "      --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_BUILD_PARALLEL_LEVEL=2"
echo
echo "Then open:  http://<this-host>:8080"
