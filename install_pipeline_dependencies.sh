#!/bin/bash
# install_pipeline_dependencies.sh
# Installs all required APT and Pip packages for:
#   1. insert_to_container.py
#   2. combine_cameras.py
#   3. gen3_complete_system.launch.py (including oak_camera_node.py)
#
# Usage:
#   bash ~/workspace/ros2_kortex_ws/install_pipeline_dependencies.sh

set -e

# Clear screen and show a modern header
clear
echo -e "\033[1;36m====================================================================\033[0m"
echo -e "\033[1;36m  ROS 2 Kortex Pipeline Dependency Installer                        \033[0m"
echo -e "\033[1;36m====================================================================\033[0m"
echo -e "\033[0;33mThis script will install all system (APT) and Python (Pip) packages\033[0m"
echo -e "\033[0;33mrequired to run the cameras, sensor fusion, and motion insertion.\033[0m"
echo ""

# -----------------------------------------------------------------------------
# 1. Sanity Checks
# -----------------------------------------------------------------------------
echo -e "\033[1;34m[1/6] Running System Verification...\033[0m"
echo "------------------------------------------------------------"

# Check if ROS 2 Humble is installed (looking in /opt/ros/humble)
if [ ! -d "/opt/ros/humble" ]; then
    echo -e "\033[1;31m[ERROR] ROS 2 Humble directory /opt/ros/humble not found.\033[0m"
    echo "This script assumes Ubuntu 22.04 LTS with ROS 2 Humble installed."
    exit 1
fi

# Try sourcing ROS 2 Humble if not already available in environment
if ! command -v ros2 &>/dev/null; then
    echo "ROS 2 not detected in environment. Attempting to source Humble..."
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
        echo "Sourced /opt/ros/humble/setup.bash successfully."
    else
        echo -e "\033[1;31m[ERROR] Could not source ROS 2 Humble setup.bash.\033[0m"
        exit 1
    fi
fi

echo -e "ROS 2 Version: \033[1;32m$(ros2 --version)\033[0m"
echo -e "Workspace Dir: \033[1;32m$(pwd)\033[0m"
echo ""

# -----------------------------------------------------------------------------
# 2. Package Manager Setup
# -----------------------------------------------------------------------------
echo -e "\033[1;34m[2/6] Updating APT Package lists and Pip...\033[0m"
echo "------------------------------------------------------------"
sudo apt-get update
sudo apt-get install -y python3-pip python3-setuptools python3-colcon-common-extensions
python3 -m pip install --upgrade pip
echo ""

# -----------------------------------------------------------------------------
# 3. System Packages (APT)
# -----------------------------------------------------------------------------
echo -e "\033[1;34m[3/6] Installing System & ROS 2 Packages (APT)...\033[0m"
echo "------------------------------------------------------------"

sudo apt-get install -y \
    python3-opencv \
    libopencv-dev \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-camera-info-manager \
    ros-humble-camera-calibration-parsers \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-tf2-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-std-msgs \
    ros-humble-visualization-msgs \
    ros-humble-shape-msgs \
    ros-humble-builtin-interfaces \
    ros-humble-control-msgs \
    ros-humble-moveit \
    ros-humble-moveit-msgs \
    ros-humble-moveit-visual-tools \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-camera-msgs \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav

echo ""

# -----------------------------------------------------------------------------
# 4. Python Packages (Pip)
# -----------------------------------------------------------------------------
echo -e "\033[1;34m[4/6] Installing Python packages (Pip)...\033[0m"
echo "------------------------------------------------------------"

# NumPy is used extensively across combine_cameras.py and oak_camera_node.py
# depthai is required for OAK-D camera node used in gen3_complete_system.launch.py
pip3 install --user numpy depthai

echo ""

# -----------------------------------------------------------------------------
# 5. Udev Rules Setup for DepthAI / OAK Cameras and RealSense
# -----------------------------------------------------------------------------
echo -e "\033[1;34m[5/6] Configuring USB Udev rules for cameras...\033[0m"
echo "------------------------------------------------------------"

# OAK camera / DepthAI udev rules
if [ ! -f /etc/udev/rules.d/80-movidius.rules ]; then
    echo "Adding Movidius/DepthAI udev rules for OAK cameras..."
    echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
    udev_reload=true
fi

# Intel RealSense udev rules (if not already installed by apt package)
if [ ! -f /etc/udev/rules.d/99-realsense-libusb.rules ]; then
    echo "Adding RealSense udev rules..."
    sudo mkdir -p /etc/udev/rules.d
    # Fetch official realsense rules if possible, or fallback to writing a generic one
    if command -v curl &>/dev/null; then
        sudo curl -sSL https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules -o /etc/udev/rules.d/99-realsense-libusb.rules
    else
        echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", MODE="0666"' | sudo tee /etc/udev/rules.d/99-realsense-libusb.rules
    fi
    udev_reload=true
fi

if [ "$udev_reload" = true ]; then
    echo "Reloading and triggering udev rules..."
    sudo udevadm control --reload-rules && sudo udevadm trigger
    echo "Udev rules reloaded successfully."
else
    echo "Udev rules already exist. Skipping."
fi
echo ""

# -----------------------------------------------------------------------------
# 6. Rosdep workspace resolution
# -----------------------------------------------------------------------------
echo -e "\033[1;34m[6/6] Syncing workspace dependencies using rosdep...\033[0m"
echo "------------------------------------------------------------"

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    echo "Initializing rosdep..."
    sudo rosdep init
fi

echo "Updating rosdep database..."
rosdep update

# Determine workspace root
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -d "$WS_ROOT/src" ]; then
    echo "Running rosdep install in workspace: $WS_ROOT"
    cd "$WS_ROOT"
    rosdep install \
        --from-paths src \
        --ignore-src \
        --rosdistro humble \
        -r -y \
        --skip-keys "depthai depthai-core python-transforms3d-pip"
else
    echo -e "\033[1;33m[WARNING] Could not locate 'src/' folder relative to this script.\033[0m"
    echo "Please make sure this script is placed in the root of your ROS 2 workspace."
fi

echo ""
echo -e "\033[1;32m====================================================================\033[0m"
echo -e "\033[1;32m  Dependencies Installed Successfully!                              \033[0m"
echo -e "\033[1;32m====================================================================\033[0m"
echo ""
echo "To build and run the pipeline:"
echo ""
echo -e "  1. Build the workspace:"
echo -e "     \033[1;37mcolcon build --executor sequential --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_BUILD_PARALLEL_LEVEL=2\033[0m"
echo ""
echo -e "  2. Source the setup script:"
echo -e "     \033[1;37msource install/setup.bash\033[0m"
echo ""
echo -e "  3. Launch the complete system:"
echo -e "     \033[1;37mros2 launch kortex_bringup gen3_complete_system.launch.py robot_ip:=192.168.1.10\033[0m"
echo ""
echo -e "  4. Run camera sensor fusion (in another terminal):"
echo -e "     \033[1;37mros2 run kortex_bringup combine_cameras.py\033[0m"
echo ""
echo -e "  5. Run container insertion action node (in another terminal):"
echo -e "     \033[1;37mros2 run kortex_bringup insert_to_container.py --ros-args -p execute_motion:=true\033[0m"
echo ""
echo -e "\033[1;36m====================================================================\033[0m"
