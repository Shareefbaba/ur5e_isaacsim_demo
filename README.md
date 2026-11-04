# UR5e Wrist Camera — Isaac Sim + ROS 2

UR5e simulation with a wrist-mounted RGB/depth camera, MoveIt control, and a five-view cube dataset with 3D-to-2D projection checks.

## Environment and compatibility

The starter project targets **ROS 2 Jazzy + Isaac Sim 6.0**. I ran it on **Ubuntu 22.04, ROS 2 Humble, and Isaac Sim 5.1**.

Changes made for this setup:
- Adapted the URDF import and ROS bridge nodes to the Isaac Sim 5.1 APIs, including resolving UR mesh paths during import.
- Updated the launch and MoveIt configuration for ROS 2 Humble; added trajectory time parameterization so planned motions execute correctly.
- Added `tool0 → camera_link → camera_optical_frame` to the robot description for the wrist camera TF.
- Mounted a USD camera and visible housing on `tool0`, and placed an 8 cm target cube in the scene.
- Published RGB, depth, and camera calibration from the same 640 × 480 camera at approximately 15 Hz in simulation time.

## Build and run

The workspace uses the original project's ROS dependencies and source packages. Install the required ROS 2 Humble, MoveIt, and Isaac Sim 5.1 dependencies first.

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
source install/local_setup.bash

ros2 launch ur5e_isaac_bridge ur5e_isaac.launch.py \
  isaac_python:=/home/shareef/miniconda3/envs/isaaclab/bin/python \
  headless:=false launch_rviz:=true
```

The `isaac_python` value above is the path used on my machine; replace it with the Python executable from your Isaac Sim 5.1 environment. Use RViz **MotionPlanning → Plan → Execute** to move the arm.

## Scripts and results

- `spawn_ur5e.py` — imports the UR5e, adds the wrist camera and target cube, and publishes robot/camera ROS topics.
- `capture_views.py` — after each manually selected and stopped RViz pose, press **Enter** to save RGB, depth, joint angles, camera TF, and intrinsics.
- `check_projection.py` — projects the known cube center using the saved camera poses and intrinsics, then draws the predicted point on each RGB image.

With the simulation running:

```bash
# From ros2_ws, after sourcing ROS 2 Humble and install/local_setup.bash
/usr/bin/python3 src/ur5e_isaac_bridge/scripts/capture_views.py --count 5
/usr/bin/python3 src/ur5e_isaac_bridge/scripts/check_projection.py
```

The five RGB captures, depth data, camera poses, calibration, and annotated projection images are in `ros2_ws/src/ur5e_isaac_bridge/resuts/`.

## Project tree

```text
ros2_ws/
├── src/
│   └── ur5e_isaac_bridge/
│       ├── launch/ur5e_isaac.launch.py
│       ├── urdf/
│       │   ├── ur_isaac.urdf.xacro
│       │   └── ur_joint_control.xacro
│       ├── scripts/
│       │   ├── spawn_ur5e.py
│       │   ├── capture_views.py
│       │   └── check_projection.py
│       └── resuts/
│           ├── rgb/             # 5 images
│           ├── depth/           # 5 .npy files
│           ├── depth_vis/       # depth previews
│           ├── projected/       # 5 images with projection markers
│           ├── poses.jsonl
│           └── camera_info.yaml
└── ur5e_isaac.repos
```

## Notes

- This implementation was tested with **Humble / Isaac Sim 5.1**, not the original Jazzy / Isaac Sim 6.0 combination.
- Viewpoints were selected manually in RViz; saving and projection checks are scripted.
- The camera, cube coordinates, and TF must remain consistent for the projection markers to align. A non-fatal XML parser warning may appear when starting the capture script.

## AI tool usage

I used **ChatGPT** to help adapt the code for ROS 2 Humble and Isaac Sim 5.1, and documentation. I ran the simulation, collected the five views, and inspected the projected results.
