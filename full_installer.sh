#!/usr/bin/env bash

set -e

# Ask the user about NVIDIA GPU usage.
NVIDIAGPU_DEFAULT="yes"
read -p "[Vizzy]: Do you have an NVIDIA graphics card in your current setup and want to use it for offloading? (yes/no) Default: $NVIDIAGPU_DEFAULT   " NVIDIAGPU_INPUT

# Use user input; if empty, use the default.
NVIDIAGPU=${NVIDIAGPU_INPUT:-$NVIDIAGPU_DEFAULT}

echo "[Vizzy]: User preference for NVIDIA GPU offloading (for alias & supporting tools): $NVIDIAGPU"

# --- Set up launch file NVIDIA preference ---
echo ""
echo "[Vizzy]: Configuring NVIDIA GPU preference for ROS 2 launch files..."

CONFIG_DIR_BASE=~/.config
VIZZY_CONFIG_DIR="$CONFIG_DIR_BASE/vizzy_installer" # Store config in a dedicated directory.
mkdir -p "$VIZZY_CONFIG_DIR"
GRAPHICS_ENV_FILE="$VIZZY_CONFIG_DIR/graphics.env"

VIZZY_USE_NVIDIA_VALUE="true" # Default to true for the launch file flag.

# Convert the NVIDIAGPU variable (yes/no) to true/false for VIZZY_USE_NVIDIA.
if [[ "${NVIDIAGPU,,}" == "yes" || "${NVIDIAGPU,,}" == "y" ]]; then
  VIZZY_USE_NVIDIA_VALUE="true"
  echo "[Vizzy]: ROS 2 launch files will be configured to attempt NVIDIA GPU offloading."
else
  echo "[Vizzy]: ROS 2 launch files will be configured NOT to attempt NVIDIA GPU offloading."
fi

# Write the choice to the environment file.
echo "# This file is managed by the Vizzy installer." > "$GRAPHICS_ENV_FILE"
echo "# It sets whether ROS 2 launch files should attempt to use NVIDIA GPU offloading." >> "$GRAPHICS_ENV_FILE"
echo "export VIZZY_USE_NVIDIA=${VIZZY_USE_NVIDIA_VALUE}" >> "$GRAPHICS_ENV_FILE"
echo "[Vizzy]: NVIDIA preference for launch files saved to $GRAPHICS_ENV_FILE"

# Ensure the environment file is sourced by .bashrc.
# Define unique markers for sed to avoid issues with special characters in paths.
BASHRC_SNIPPET_MARKER_START="# VIZZY_INSTALLER_GRAPHICS_CONFIG_LAUNCH_START"
BASHRC_SNIPPET_MARKER_END="# VIZZY_INSTALLER_GRAPHICS_CONFIG_LAUNCH_END"
BASHRC_LINE_TO_ADD="if [ -f \"$GRAPHICS_ENV_FILE\" ]; then source \"$GRAPHICS_ENV_FILE\"; fi"

# Remove old block if it exists to prevent duplicates, creating a backup .bashrc.bak.
if grep -qF "$BASHRC_SNIPPET_MARKER_START" ~/.bashrc; then
    sed -i.bak "/$BASHRC_SNIPPET_MARKER_START/,/$BASHRC_SNIPPET_MARKER_END/d" ~/.bashrc
fi

# Add the new block.
echo "" >> ~/.bashrc # Ensure it's on a new line if .bashrc doesn't end with one.
echo "$BASHRC_SNIPPET_MARKER_START" >> ~/.bashrc
echo "$BASHRC_LINE_TO_ADD" >> ~/.bashrc
echo "$BASHRC_SNIPPET_MARKER_END" >> ~/.bashrc
echo "[INFO] Graphics configuration sourcing for launch files updated in ~/.bashrc."
echo "[INFO] Please run 'source ~/.bashrc' or open a new terminal for this change to take full effect for new launches."


echo ""
echo "----------------------------------------------------------------------"
echo "[Vizzy]: Starting ROS 2 Humble & Ignition Gazebo Fortress Installation"
echo "----------------------------------------------------------------------"
echo ""

# [1] Ensure locale is UTF-8.
echo "[Step 1] Configuring locale..."
sudo apt update
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
echo ""

# [2] Enable universe repo.
echo "[Step 2] Enabling universe repository..."
sudo apt install -y software-properties-common
sudo add-apt-repository universe -y # Added -y to prevent interactive prompt.
echo ""

# [3] Add ROS 2 APT repository.
echo "[Step 3] Adding ROS 2 APT repository..."
sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
echo ""

# [4] Install ROS 2.
echo "[Step 4] Installing ROS 2 Humble Desktop..."
sudo apt update
sudo apt upgrade -y
sudo apt install -y ros-humble-desktop ros-dev-tools ros-humble-joint-state-publisher
echo ""

# [5] Add to .bashrc.
echo "[Step 5] Adding ROS 2 sourcing to ~/.bashrc..."
ROS_SETUP="source /opt/ros/humble/setup.bash"
if ! grep -Fxq "$ROS_SETUP" ~/.bashrc; then
  echo "$ROS_SETUP" >> ~/.bashrc
  echo "[INFO] Successfully added ROS 2 sourcing to ~/.bashrc"
fi
source /opt/ros/humble/setup.bash
echo ""

# [6] Install Gazebo Fortress (Ignition Fortress).
echo "[Step 6] Installing Ignition Gazebo Fortress..."
sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | \
  sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update
sudo apt install -y ignition-fortress
echo ""

# [7–8] Optional: NVIDIA GPU setup for command-line alias and supporting tools.
# This uses the NVIDIAGPU variable set at the beginning.
if [[ "${NVIDIAGPU,,}" == "yes" || "${NVIDIAGPU,,}" == "y" ]]; then
  echo "[Step 7] Installing Mesa utilities (for NVIDIA graphics card usage)..."
  sudo apt install -y mesa-utils
  echo ""
  echo "----------------------------------------------------------------------"
  echo "[Step 8] Creating alias 'ign-nvidia' for GPU offloading..."
  echo "----------------------------------------------------------------------"
  echo ""
  IGN_ALIAS_MARKER_START="# VIZZY_INSTALLER_IGN_NVIDIA_ALIAS_START"
  IGN_ALIAS_MARKER_END="# VIZZY_INSTALLER_IGN_NVIDIA_ALIAS_END"
  IGN_ALIAS_LINE="alias ign-nvidia='__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json ign gazebo'"
  
  # Remove old block if it exists.
  if grep -qF "$IGN_ALIAS_MARKER_START" ~/.bashrc; then
      sed -i.bak "/$IGN_ALIAS_MARKER_START/,/$IGN_ALIAS_MARKER_END/d" ~/.bashrc
  fi

  # Add new block.
  echo "" >> ~/.bashrc
  echo "$IGN_ALIAS_MARKER_START" >> ~/.bashrc
  echo "$IGN_ALIAS_LINE" >> ~/.bashrc
  echo "$IGN_ALIAS_MARKER_END" >> ~/.bashrc
  echo "[INFO] Added/Updated NVIDIA offload alias 'ign-nvidia' to ~/.bashrc"
  echo ""
fi

# [9] Install PCL from binary.
echo "[Step 9] Installing PCL dependencies..."
sudo apt install -y libpcl-dev
sudo apt install -y libpcap-dev
echo ""

# [10] Install ros_gz from binary.
echo "[Step 10] Installing ros_gz dependencies..."
# The duplicate repo source lines are correctly commented out from your previous version.
sudo apt-get update
sudo apt install -y ros-humble-ros-gz # Added -y for consistency.
echo ""

# [11] Setup colcon workspace and clone repo.
COLCON_WS=~/vizzy2_ws
read -p "[Vizzy]: Enter the path for the colcon workspace (Default: $COLCON_WS): " NEW_COLCON_WS
if [ ! -z "$NEW_COLCON_WS" ]; then
  COLCON_WS=$NEW_COLCON_WS
fi
echo "[Step 11] Setting up colcon workspace at $COLCON_WS..."
# Define the target src directory.
TARGET_SRC_DIR="$COLCON_WS/src"
# Ensure the src directory exists.
mkdir -p "$TARGET_SRC_DIR"
# Change to the src directory to perform the clone.
cd "$TARGET_SRC_DIR"
# Check if the current directory (src) is already a git repository.
# We look for the .git folder which indicates the root of a git repository.
if [ ! -d ".git" ]; then
  echo "[INFO] vizzy2.0 repository not found or not initialized in $TARGET_SRC_DIR."
  echo "[INFO] Cloning vizzy2.0 repository contents directly into $TARGET_SRC_DIR..."
  git clone https://github.com/vislab-tecnico-lisboa/vizzy2.0.git .
  echo "[INFO] Successfully cloned vizzy2.0 contents into $TARGET_SRC_DIR."
else
  echo "[INFO] Directory $TARGET_SRC_DIR already appears to be a git repository. Attempting to update..."
  git pull # Update the repository if it already exists.
fi
echo ""

# [12] Install dependencies.
cd "$COLCON_WS" # Ensure we are in the workspace root.
echo "[Step 12] Installing ROS 2 package dependencies via rosdep..."
sudo apt install -y python3-rosdep
sudo rosdep init || true  # Will fail if already done.
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble 
echo ""

# [13] Build workspace.
echo "[Step 13] Building workspace with colcon..."
colcon build --cmake-clean-cache
echo ""

# [14] Source overlay workspace.
echo "[Step 14] Adding workspace to ~/.bashrc..."
OVERLAY_SETUP_LINE="source $COLCON_WS/install/local_setup.bash"
OVERLAY_SETUP_MARKER_START="# VIZZY_INSTALLER_COLCON_WS_SETUP_START"
OVERLAY_SETUP_MARKER_END="# VIZZY_INSTALLER_COLCON_WS_SETUP_END"

# Remove old block if it exists.
if grep -qF "$OVERLAY_SETUP_MARKER_START" ~/.bashrc; then
    sed -i.bak "/$OVERLAY_SETUP_MARKER_START/,/$OVERLAY_SETUP_MARKER_END/d" ~/.bashrc
fi

# Add new block.
echo "" >> ~/.bashrc
echo "$OVERLAY_SETUP_MARKER_START" >> ~/.bashrc
echo "$OVERLAY_SETUP_LINE" >> ~/.bashrc
echo "$OVERLAY_SETUP_MARKER_END" >> ~/.bashrc
echo "[INFO] Workspace overlay sourcing added/updated in ~/.bashrc"
echo ""

echo "-------------------------------------------------------------------------------------------------------------------"
echo "[Vizzy]: ROS 2 Humble, Gazebo & Vizzy2.0 Setup Complete!"
echo "## Please run 'source ~/.bashrc' or open a new terminal for all changes to take effect! ##"

# Updated final information block using VIZZY_USE_NVIDIA_VALUE.
if [[ "$VIZZY_USE_NVIDIA_VALUE" == "true" ]]; then
  echo "[INFO] Your system is configured to attempt using the NVIDIA GPU for ROS 2 launch files."
  if [[ "${NVIDIAGPU,,}" == "yes" || "${NVIDIAGPU,,}" == "y" ]]; then # Check if alias was also installed.
    echo "[INFO] For manual command-line Ignition Gazebo with NVIDIA, you can use the 'ign-nvidia' alias."
  fi
else
  echo "[INFO] Your system is configured to use default graphics for ROS 2 launch files."
  echo "[INFO] For manual command-line Ignition Gazebo, use the 'ign gazebo' command."
fi
echo "[INFO] To run the Vizzy Gazebo Simulation, try the command: 'ros2 launch vizzy_launch vizzy_simulation_launch.xml'"
echo "-------------------------------------------------------------------------------------------------------------------"
