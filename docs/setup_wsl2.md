# Setup Guide: WSL2 + ROS Noetic + Gazebo 11

This guide covers the complete setup process for running the UAV LiDAR Autonomy project on Windows 11 using WSL2 with Ubuntu 20.04.

---

## Prerequisites

- Windows 11 (Build 22000 or later)
- WSL2 enabled with Ubuntu 20.04 installed
- At least 8 GB RAM (16 GB recommended for Gazebo + RViz)
- A CPU with 4+ cores

---

## Step 1 — Install Ubuntu 20.04 on WSL2

```powershell
# Run in PowerShell (Administrator)
wsl --install -d Ubuntu-20.04
```

After installation, launch Ubuntu from the Start menu and complete initial user setup.

---

## Step 2 — Install ROS Noetic

```bash
# Set up sources
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-noetic.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Update and install full desktop installation
sudo apt-get update
sudo apt-get install -y ros-noetic-desktop-full

# Environment setup
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install build tools
sudo apt-get install -y python3-rosdep python3-rosinstall python3-rosinstall-generator \
  python3-wstool build-essential python3-catkin-tools

# Initialise rosdep
sudo rosdep init
rosdep update
```

---

## Step 3 — Install Project Dependencies

```bash
sudo apt-get install -y \
  ros-noetic-velodyne-simulator \
  ros-noetic-velodyne-description \
  ros-noetic-velodyne-gazebo-plugins \
  ros-noetic-velodyne-laserscan \
  ros-noetic-pointcloud-to-laserscan \
  ros-noetic-hector-slam \
  ros-noetic-hector-mapping \
  ros-noetic-tf2-tools \
  ros-noetic-rqt-graph \
  ros-noetic-teleop-twist-keyboard

pip3 install scipy numpy
```

---

## Step 4 — Display Configuration (WSLg)

Windows 11 includes **WSLg** (Windows Subsystem for Linux GUI), which automatically provides a virtual display for Linux GUI applications. No additional X server software (VcXsrv, Xming) is needed on Windows 11.

Verify your display is configured:

```bash
echo $DISPLAY
# Expected: :0
```

If `$DISPLAY` is empty, ensure WSLg is enabled:

```powershell
# In PowerShell
wsl --update
wsl --shutdown
```

Then relaunch your Ubuntu terminal.

### GPU acceleration

Gazebo 11 works best with GPU acceleration. In WSL2, the GPU driver is bridged through the Windows DirectX layer. If Gazebo fails to start or renders blank:

```bash
export LIBGL_ALWAYS_SOFTWARE=1
```

Add this to `~/.bashrc` for persistence if needed.

> **Note**: `LIBGL_ALWAYS_SOFTWARE=1` forces Mesa software rendering. Gazebo will run at reduced frame rate but correctly. Remove this flag once your WSL2 GPU driver is confirmed working.

---

## Step 5 — Build and Run

```bash
cd ~/robotics_projects/uav_lidar_stack

# Set display
export DISPLAY=:0
export LIBGL_ALWAYS_SOFTWARE=1

# Source environment
source scripts/setup.sh

# Build
./scripts/build.sh

# Launch
roslaunch uav_lidar_bringup uav_lidar_system.launch
```

---

## Troubleshooting

### Gazebo opens but shows a black/blank window

- Try `export LIBGL_ALWAYS_SOFTWARE=1` before launching
- Check WSL2 GPU driver: `ls /dev/dxg` should exist on Windows 11

### `roslaunch` hangs after "process[gazebo-1]: started"

- Gazebo can take 30–60 seconds to start on first run (loading physics engine + world)
- If it hangs indefinitely: kill it and try `export LIBGL_ALWAYS_SOFTWARE=1`

### No window appears at all

- Confirm `$DISPLAY` is set: `echo $DISPLAY` should print `:0`
- On older Windows 10 WSL2: install VcXsrv from the Windows side, start it with `-multiwindow -ac`, then set `export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0.0`

### `[rosrun] Couldn't find executable named ...`

The workspace hasn't been built or sourced:

```bash
cd ~/robotics_projects/uav_lidar_stack/catkin_ws
catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

### `apt-get update` hangs for a long time

This is a known WSL2 issue with `systemd-resolved`. Try:

```bash
sudo apt-get -o Acquire::ForceIPv4=true update
```

### Gazebo physics runs slower than real time

- Reduce the VLP-16 scan rate in the URDF (`<update_rate>`) from 10 to 5 Hz
- Disable Gazebo shadows: add `<shadows>false</shadows>` under `<scene>` in `warehouse.world`
- Close other applications consuming GPU/CPU resources

---

## Recommended `~/.bashrc` additions

```bash
# ROS Noetic
source /opt/ros/noetic/setup.bash

# UAV LiDAR project workspace
source ~/robotics_projects/uav_lidar_stack/catkin_ws/devel/setup.bash

# WSL2 display
export DISPLAY=:0
export LIBGL_ALWAYS_SOFTWARE=1
```

After editing, apply with `source ~/.bashrc`.
