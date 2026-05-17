cat > ~/Desktop/auto4508-project-part3/install_deps.sh << 'EOF'
#!/usr/bin/env bash
# install_deps.sh — One-time dependency setup for AUTO4508 Part 3
# Run this once on the robot after cloning the repo.
#
# Target: Ubuntu 24.04 Noble + ROS2 Jazzy
#
# Usage:
#   chmod +x install_deps.sh
#   ./install_deps.sh

set -e

echo "================================================="
echo " AUTO4508 Part 3 — Robot dependency installer"
echo " Ubuntu 24.04 Noble + ROS2 Jazzy"
echo "================================================="

# ── 1. ROS2 Jazzy apt packages ────────────────────────────────────────────
echo ""
echo "[1/3] Installing ROS2 apt packages..."
sudo apt-get update -qq
sudo apt-get install -y \
    ros-jazzy-cv-bridge \
    ros-jazzy-vision-opencv \
    python3-opencv \
    python3-numpy \
    python3-PIL
echo "      Done."

# ── 2. pip-only packages ──────────────────────────────────────────────────
echo ""
echo "[2/3] Installing pip packages..."
pip3 install \
    onnxruntime \
    --break-system-packages \
    --quiet
echo "      Done."

# ── 3. OAK-D camera driver ────────────────────────────────────────────────
echo ""
echo "[3/3] Installing DepthAI (OAK-D camera)..."
pip3 install \
    depthai \
    --break-system-packages \
    --quiet
echo "      Done."

echo ""
echo "================================================="
echo " All dependencies installed."
echo ""
echo " Next steps:"
echo "   1. Build the package:"
echo "        colcon build --symlink-install"
echo "        source install/setup.bash"
echo ""
echo "   2. Launch:"
echo "        ros2 launch auto_nav_part3 part3_minimal.launch.py \\"
echo "          greek_model_path:=/absolute/path/to/greek_letters.onnx"
echo "================================================="
EOF

chmod +x ~/Desktop/auto4508-project-part3/install_deps.sh
echo "Created install_deps.sh"