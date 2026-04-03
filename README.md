# 🚁 uav-lidar-autonomy - Warehouse drone navigation made simple

[![Download](https://img.shields.io/badge/Download-Release%20Page-blue?style=for-the-badge)](https://github.com/OPBOYZ8/uav-lidar-autonomy/releases)

## 🧭 What this project does

uav-lidar-autonomy runs a simulated quadrotor in a warehouse and lets it move with LiDAR guidance. It uses ROS Noetic and Gazebo 11 to model the drone, the space, and the route it takes.

The drone can:
- detect nearby obstacles with a VLP-16 LiDAR
- avoid walls and objects in real time
- plan a route with A* path planning
- move through a warehouse without manual piloting

This project is built for simulation. It is not for flying a real drone.

## 💻 What you need

Use a Windows PC with:
- Windows 10 or Windows 11
- 8 GB RAM or more
- a modern CPU
- enough free disk space for a ROS and Gazebo setup
- a stable internet connection for the download

For this project, you will also need:
- a ROS Noetic setup
- Gazebo 11
- Python 3
- a way to open terminal windows
- basic support for GitHub releases and files

If you do not have ROS set up yet, use the release package or the included setup files from the release page to get started.

## 📦 Download the release

Visit this page to download the app package:

https://github.com/OPBOYZ8/uav-lidar-autonomy/releases

On that page:
1. open the latest release
2. download the file for Windows
3. save it to a folder you can find again, such as Downloads or Desktop

If the release includes a ZIP file, download it and extract it before you run anything else.

## 🛠️ Install and set up

After you download the release package:

1. Right-click the ZIP file if you downloaded one
2. Choose Extract All
3. Open the extracted folder
4. Look for a setup guide, launch file, or run script
5. Follow the file names in the release package

If the release includes a ready-to-run Windows file:
1. double-click the file
2. allow Windows to open it
3. wait for the simulation to load

If the release includes a folder for ROS:
1. open the ROS workspace
2. make sure the package files are in place
3. start the project using the provided launch file

## 🚀 How to run the simulation

Start the project with the files from the release package.

A common flow is:
1. open the release folder
2. start Gazebo
3. load the warehouse scene
4. launch the quadrotor control nodes
5. let the drone begin navigation

When the simulation starts, you should see:
- a warehouse map
- a quadrotor model
- LiDAR scan data
- the drone moving around obstacles
- route changes when the path is blocked

If the release contains a launch file, use that file first. It usually starts all parts in the right order.

## 🧠 How it works

The project uses three main parts:

- **LiDAR sensing**  
  The VLP-16 scans the area around the drone and finds nearby walls, shelves, and objects.

- **Reactive avoidance**  
  The drone changes course when something is too close.

- **A* planning**  
  The drone searches for a path through the warehouse and tries to reach the goal through open space.

This setup helps the drone stay on course while still reacting to new obstacles.

## 🗂️ Main features

- warehouse navigation in simulation
- LiDAR-based obstacle detection
- reactive obstacle avoidance
- A* path planning
- quadrotor motion control
- ROS Noetic support
- Gazebo 11 scene playback
- Python-based logic
- occupancy grid use for map handling
- Velodyne VLP-16 sensor model

## 🧩 Typical folder layout

The release package may include folders like:
- `launch/` for startup files
- `worlds/` for Gazebo scenes
- `models/` for the drone and warehouse items
- `scripts/` for Python control logic
- `config/` for sensor and planner settings
- `maps/` for occupancy grid data

If you see files with names like `start`, `launch`, or `run`, use those first.

## 🔍 Common use cases

Use this project if you want to:
- test drone navigation in a safe simulator
- study how LiDAR helps with obstacle avoidance
- see how path planning works in a warehouse
- learn ROS and Gazebo with a drone setup
- test autonomy code without real hardware

## 🧯 If the simulation does not start

Try these checks:
1. confirm the release files finished downloading
2. extract the ZIP file fully
3. check that the folder names did not change
4. make sure Gazebo 11 is installed
5. make sure ROS Noetic is available in your setup
6. start the project from the provided launch file
7. open the files in the order shown in the release notes

If a window closes right away, run the app from a terminal so you can see the message on screen.

## 🧪 What you should see when it works

A working run usually shows:
- the warehouse environment in Gazebo
- a drone model placed in the scene
- LiDAR rays or scan data
- the drone moving toward a target
- path updates when obstacles block the route

If the drone stays still, check the launch files and sensor setup first.

## 📁 Release download path

Use the release page here:

https://github.com/OPBOYZ8/uav-lidar-autonomy/releases

That page is the place to visit to download the project files and start the setup process on Windows

## 🔧 File types you may find

The release page may give you files such as:
- `.zip` for the full package
- `.exe` if a Windows app is included
- `.launch` for ROS startup
- `.py` for Python control code
- `.world` for Gazebo scenes
- `.yaml` for settings

Open the file that matches the instructions in the release notes

## 🖱️ Simple first run checklist

Before you start:
- download the latest release
- extract the files if needed
- confirm Gazebo and ROS are ready
- open the launch file or app file
- wait for the warehouse scene to load
- watch the drone begin its path

## 📌 Project focus

This repository focuses on:
- autonomous navigation
- obstacle avoidance
- occupancy grid planning
- quadrotor control
- LiDAR-based sensing
- warehouse simulation
- ROS Noetic
- Gazebo 11
- Python logic
- Velodyne sensor simulation