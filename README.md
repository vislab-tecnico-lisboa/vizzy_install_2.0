# Vizzy 2.0 System Setup Installer

[![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04%20LTS-orange)](https://releases.ubuntu.com/22.04/)
[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble%20Hawksbill-blue)](https://docs.ros.org/en/humble/index.html)
[![Gazebo Fortress](https://img.shields.io/badge/Gazebo-Fortress-blueviolet)](https://gazebosim.org/docs/fortress)
This repository provides automated scripts to configure a compatible Ubuntu 22.04 (Jammy Jellyfish) system for development and simulation with the [Vizzy 2.0 Humanoid Robot](https://github.com/vislab-tecnico-lisboa/vizzy2.0). The installer will set up ROS 2 Humble Hawksbill, Ignition Gazebo Fortress, and all necessary dependencies, including the `vizzy2.0` colcon workspace.

## Overview

The `full_installer.sh` script automates the following key setup processes:
* System locale configuration (UTF-8).
* Addition of required package repositories (ROS 2, Gazebo).
* Installation of ROS 2 Humble Desktop and development tools.
* Installation of Ignition Gazebo Fortress.
* Optional NVIDIA GPU offloading configuration for Gazebo.
* Installation of Point Cloud Library (PCL) and `ros_gz` bridge dependencies.
* Cloning of the `vizzy2.0` repository.
* Installation of `vizzy2.0` dependencies using `rosdep`.
* Building the `vizzy2.0` colcon workspace.
* Configuration of the `.bashrc` file for sourcing ROS 2 and the new workspace.

## Prerequisites

Before running the installer, please ensure your system meets the following requirements:
* **Operating System:** Ubuntu 22.04 LTS (Jammy Jellyfish).
* **Internet Connection:** Required for downloading packages and cloning repositories.
* **Git:** The `git` command-line tool must be installed (`sudo apt install git`).
* **Sudo Privileges:** The installer requires `sudo` access to install system packages and configure repositories.

## Installation

Follow these steps to set up your system:

1.  **Clone this Repository:**
    Open a terminal and clone this `vizzy_install_2.0` repository to your local machine.
    ```bash
    git clone https://github.com/vislab-tecnico-lisboa/vizzy_install_2.0.git
    cd vizzy_install_2.0
    ```

2.  **Make the Installer Executable:**
    Navigate into the cloned directory and give the installer script execute permissions.
    ```bash
    chmod +x full_installer.sh
    ```

3.  **Run the Installer Script:**
    Execute the script from within the `vizzy_install_2.0` directory.
    ```bash
    ./full_installer.sh
    ```

## Installer Configuration Prompts

During the execution, the script will prompt you for a few configuration choices:

1.  **NVIDIA GPU Offloading:**
    * **Prompt:** `[Vizzy]: Do you have an NVIDIA graphics card in your current setup and want to use it for offloading? (yes/no) Default: yes`
    * **Purpose:** If you have an NVIDIA graphics card, choosing "yes" (or pressing Enter for the default) will configure your system to attempt to use the NVIDIA GPU for running Ignition Gazebo simulations. This is achieved by setting specific environment variables that enable PRIME render offloading.
    * **Note:** Your system must have a functional NVIDIA driver and a compatible graphics card for this to work. If you choose "no" or do not have an NVIDIA card, Gazebo will use the default graphics renderer (often an integrated GPU). This choice is saved and applied to ROS 2 launch files. An alias `ign-nvidia` will also be added to your `.bashrc` for manual command-line offloading if "yes" is selected.

2.  **Sudo Password:**
    * The script will request your user password via `sudo` once at the beginning of the execution.
    * **Purpose:** This is required for system-level operations such as installing packages via `apt`, adding repositories, and initializing `rosdep`.

3.  **Colcon Workspace Path:**
    * **Prompt:** `[Vizzy]: Enter the path for the colcon workspace (Default: ~/vizzy2_ws):`
    * **Purpose:** This determines the directory where the `vizzy2.0` source code will be cloned and built.
    * **Default:** If you press Enter without typing a path, it will use `~/vizzy2_ws` (a directory named `vizzy2_ws` in your home folder).

## Post-Installation

After the installer script completes successfully:

1.  **Regarding a PCL-related Warning (Benign):**
    * You may have observed a warning message similar to `** WARNING ** io features related to pcap will be disabled` during the `colcon build` phase (Step 13) of the installation.
    * **This is a known, benign warning related to the Point Cloud Library (PCL).** Crucially, this specific warning does not affect the core functionality of Vizzy's simulation or navigation capabilities as set up by this installer. However, we are continuously working to refine the setup process and minimize all warnings :).
  
2.  **Update Your Shell Environment:**
    * **IMPORTANT:** For all environment changes, newly set aliases, and sourced paths to take effect, you **must** either source your updated `.bashrc` file in your current terminal.
      ```bash
      source ~/.bashrc
      ```
      *(Alternatively, simply close your current terminal and open a new one.)*

## Usage

Once the setup is complete and your terminal environment is updated:

* **To launch the Vizzy Gazebo Simulation:**
    ```bash
    ros2 launch vizzy_launch vizzy_simulation_launch.xml
    ```

* **NVIDIA GPU Offloading (if enabled during installation):**
    * The launch command above should automatically attempt to use the NVIDIA GPU if you selected "yes" during installation.
    * For manually running Ignition Gazebo with NVIDIA offloading (outside of ROS 2 launch if needed), you can use the alias:
        ```bash
        ign-nvidia gazebo <your_world_or_other_args>
        ```
    * To run Gazebo normally (without forced NVIDIA offloading via the alias):
        ```bash
        ign gazebo <your_world_or_other_args>
        ```

## Troubleshooting

* **`libEGL warning: egl: failed to create dri2 screen` (or similar EGL/DRI2 errors with Gazebo):**
    This error usually indicates an issue with how Gazebo is interacting with your graphics drivers, especially when NVIDIA offloading is intended.
    1.  **If you selected NVIDIA support during installation:**
        * **Verify NVIDIA Drivers:** Ensure your proprietary NVIDIA drivers are correctly installed and functioning. You can often check this with the command `nvidia-smi`.
        * **Check Offloading Configuration:** The installer sets up an environment variable to tell launch files to use NVIDIA. Verify this is active in your current terminal:
            ```bash
            echo $VIZZY_USE_NVIDIA
            ```
            This command should output `true`.
        * **Terminal Session:** If it doesn't output the expected value, ensure you have run `source ~/.bashrc` in your current terminal or opened a new terminal after the installer finished.
        * **Re-run Installer / Manual Check:** If the `VIZZY_USE_NVIDIA` variable is not being set correctly despite sourcing `.bashrc`, you can try re-running the installer's graphics setup portion or manually check the contents of `~/.config/vizzy_installer/graphics.env` and your `~/.bashrc` for the sourcing line.
        * **Still an issue?** If the variable is `true` and you still see EGL errors, there might be a deeper issue with your NVIDIA driver installation, the specific offloading variables, or how they are being applied by the launch file. Consider opening an issue in this repository with details about your GPU, driver version, and the exact error messages.
    2.  **If you did NOT select NVIDIA support (or chose "no"):**
        * Ensure your system's Mesa drivers for integrated graphics (Intel/AMD) are up-to-date:
            ```bash
            sudo apt update && sudo apt upgrade
            ```
        * If errors persist, it might be related to a specific Gazebo version or system configuration. Please open an issue with details.
* **Package not found / Command not found:** Ensure you have sourced `~/.bashrc` in your current terminal or opened a new terminal after the installation.
* **Problems with `sudo apt-get upgrade -y`:** This issue can occur during the `ROS 2 installation` phase (Step 4) of the installation. To resolve this, just run on the terminal:
  ```bash
  sudo apt-get update --fix-missing
  ```
    and then you can restart the installation process with:
    ```bash
    ./full_installer.sh
    ```
