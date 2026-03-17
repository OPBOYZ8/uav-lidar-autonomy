# Images

This directory holds screenshots, diagrams, and demo recordings for the README and documentation.

## Needed assets

| Filename | Description | How to capture |
|----------|-------------|----------------|
| `warehouse_overview.png` | Gazebo viewport showing the full warehouse with racks, boxes, walls, and pillars | Take a screenshot of Gazebo after launching `uav_lidar_system.launch` |
| `lidar_scan_rviz.png` | RViz view showing the `/scan` LaserScan (red dots) and `/cloud_filtered` point cloud overlaid on the drone | Grab from the RViz window after the drone starts moving |
| `occupancy_map_rviz.png` | RViz view showing the `/map` OccupancyGrid after partial exploration | Wait ~30 s for the map to build, then screenshot |
| `navigation_path_rviz.png` | RViz view showing the `/uav/path` A\* planned path overlaid on the occupancy map | Screenshot once the A\* planner publishes its first path (~5 s after start) |
| `system_architecture.png` | Clean diagram of the full data-flow pipeline | Export from draw.io or equivalent using the ASCII diagram in `docs/architecture.md` as a reference |
| `demo.gif` | Animated GIF of a complete navigation run | Record the RViz window during a full mission using `kazam`, `peek`, or OBS |

## Capture tips

To screenshot inside WSL2, you can use `scrot` or take a Windows screenshot (Win+Shift+S) and save the result here.

For a demo GIF, record at ~15 fps and keep it under 20 MB. Tools:
- `peek` (Linux GUI screen recorder) — `sudo apt install peek`
- OBS Studio (Windows) recording the WSLg window
- `ffmpeg` to convert a video to GIF: `ffmpeg -i demo.mp4 -vf fps=15,scale=800:-1 demo.gif`
