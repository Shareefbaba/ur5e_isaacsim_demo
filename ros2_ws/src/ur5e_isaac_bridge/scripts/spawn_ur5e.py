
# Spawn the official UR description into Isaac Sim and publish joint state
# over the ROS 2 bridge. Run with Isaac Sim's Python, for example:
#   ~/isaacsim/python.sh spawn_ur5e.py --xacro /path/ur_isaac.urdf.xacro

import argparse
import os
import subprocess
import sys
import tempfile

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--xacro", required=True, help="Path to ur_isaac.urdf.xacro")
parser.add_argument("--ur-type", default="ur5e")
parser.add_argument("--joint-commands-topic", default="/isaac_joint_commands")
parser.add_argument("--joint-states-topic", default="/isaac_joint_states")
parser.add_argument("--headless", action="store_true")
args, _unknown = parser.parse_known_args()

from isaacsim import SimulationApp

simulation_app = SimulationApp(
    {"renderer": "RealTimePathTracing", "headless": args.headless}
)

import numpy as np
import omni.graph.core as og
import usdrt.Sdf

# Isaac Sim 5.1 compatibility: use the command-based URDF importer.
import omni.kit.commands

# Isaac Sim 5.1 compatibility: extension management and timeline playback.
from isaacsim.core.utils.extensions import enable_extension
import omni.timeline

from isaacsim.core.experimental.utils import stage as stage_utils

# Isaac Sim 5.1 compatibility: set the GUI viewport using the supported utility.
from isaacsim.core.utils.viewports import set_camera_view

from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.storage.native import get_assets_root_path
from pxr import Usd, UsdPhysics, UsdGeom, Gf
import omni.usd


def expand_urdf() -> str:
    """Expand the official xacro with absolute mesh paths for the importer."""
    out_dir = tempfile.mkdtemp(prefix="ur5e_isaac_urdf_")
    urdf_path = os.path.join(out_dir, "ur5e.urdf")
    command = [
        "xacro",
        args.xacro,
        f"ur_type:={args.ur_type}",
        "name:=ur",
        "force_abs_paths:=true",
        f"joint_commands_topic:={args.joint_commands_topic}",
        f"joint_states_topic:={args.joint_states_topic}",
    ]

    result = subprocess.run(
        command,
        check=False,
        text=True,
        capture_output=True,
    )

    if result.returncode != 0:
        sys.stderr.write(result.stderr)
        raise RuntimeError(
            "xacro failed. Source the ROS 2 workspace before launching Isaac Sim."
        )

    with open(urdf_path, "w", encoding="utf-8") as urdf_file:
        urdf_file.write(result.stdout)

    return urdf_path


def articulation_path(hint: str) -> str:
    stage = stage_utils.get_current_stage(backend="usd")
    prim = stage.GetPrimAtPath(hint)

    if prim and prim.IsValid() and prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        return hint

    if prim and prim.IsValid():
        for child in Usd.PrimRange(prim):
            if child.HasAPI(UsdPhysics.ArticulationRootAPI):
                return str(child.GetPath())

    return hint


def main() -> None:

    enable_extension("isaacsim.ros2.bridge")
    enable_extension("isaacsim.asset.importer.urdf")
    simulation_app.update()

    stage_utils.set_stage_units(meters_per_unit=1.0)

    # Isaac Sim 5.1: the graphical viewport is not needed in headless mode.
    # This is the viewer camera, NOT the wrist-mounted RGB sensor.
    if not args.headless:
        set_camera_view(
            eye=np.array([1.6, 1.6, 1.2]),
            target=np.array([0.0, 0.0, 0.4]),
            camera_prim_path="/OmniverseKit_Persp",
        )

    assets_root = get_assets_root_path()
    if assets_root:
        stage_utils.add_reference_to_stage(
            assets_root + "/Isaac/Environments/Simple_Room/simple_room.usd",
            "/background",
        )

    urdf_path = expand_urdf()

    # Isaac Sim 5.1 mesh-path compatibility fix:
    # The Humble UR xacro leaves package://ur_description mesh URLs in the URDF.
    # Convert them to absolute paths for the Isaac Sim URDF importer.
    # This changes ONLY the temporary URDF generated above; ROS / MoveIt
    # continue to use the original Xacro description.
    with open(urdf_path, "r", encoding="utf-8") as urdf_file:
        urdf_xml = urdf_file.read()

    original_mesh_prefix = "package://ur_description/"
    absolute_mesh_prefix = "/opt/ros/humble/share/ur_description/"

    if original_mesh_prefix not in urdf_xml:
        raise RuntimeError(
            "Expected UR mesh URLs missing from generated URDF: "
            + original_mesh_prefix
        )

    if not os.path.isdir(absolute_mesh_prefix):
        raise RuntimeError(
            "UR description mesh directory does not exist: "
            + absolute_mesh_prefix
        )

    urdf_xml = urdf_xml.replace(original_mesh_prefix, absolute_mesh_prefix)

    with open(urdf_path, "w", encoding="utf-8") as urdf_file:
        urdf_file.write(urdf_xml)

    print(
        "[ur5e_isaac] Resolved package://ur_description/ mesh URLs for Isaac Sim",
        flush=True,
    )

    usd_dir = tempfile.mkdtemp(prefix="ur5e_isaac_usd_")
    usd_path = os.path.join(usd_dir, "ur5e.usd")

    # Isaac Sim 5.1 compatibility: use Kit commands to import the URDF.
    success, import_config = omni.kit.commands.execute(
        "URDFCreateImportConfig"
    )

    if not success:
        raise RuntimeError("Failed to create URDF import configuration")

    import_config.fix_base = True
    import_config.merge_fixed_joints = False
    import_config.collision_from_visuals = False
    import_config.make_default_prim = True

    success, _ = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=urdf_path,
        import_config=import_config,
        dest_path=usd_path,
    )

    if not success:
        raise RuntimeError("Failed to import UR5e URDF")

    robot_root = "/ur5e"
    stage_utils.add_reference_to_stage(usd_path, robot_root)
    
    # Problem 1 - Point 1:
    # Mount a USD RGB camera on the UR5e tool0 link.
    # The camera inherits tool0's motion.

    stage = omni.usd.get_context().get_stage()

    tool0_path = f"{robot_root}/tool0"
    tool0_prim = stage.GetPrimAtPath(tool0_path)

    if not tool0_prim.IsValid():
        raise RuntimeError(f"Cannot mount camera: {tool0_path} not found")

    wrist_camera_path = f"{tool0_path}/wrist_cam"
    wrist_camera = UsdGeom.Camera.Define(stage, wrist_camera_path)
    wrist_camera.CreateFocalLengthAttr(15.0)
    
    wrist_camera.GetClippingRangeAttr().Set(
       Gf.Vec2f(0.01, 1000.0)
    )

    # Temporary mounting offset: 5 cm along tool0's local Z axis.
    UsdGeom.Xformable(wrist_camera.GetPrim()).AddTranslateOp().Set(
        Gf.Vec3d(0.0, 0.0, 0.05)
    )
    
    UsdGeom.Xformable(wrist_camera.GetPrim()).AddRotateXYZOp().Set(
        Gf.Vec3f(180.0, 0.0, -90.0)
    )
    
    # The RGB sensor remains separate from this visual geometry.
    housing = UsdGeom.Cube.Define(stage, f"{tool0_path}/wrist_cam_housing")
    housing.CreateSizeAttr(0.04)
    housing.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.020))
    housing.CreateDisplayColorAttr([Gf.Vec3f(0.1, 0.1, 0.1)])
    
    # Problem 2: Target cube for the five-viewpoint dataset.
    cube_center = (0.50, 0.0, 0.04)  # World coordinates, metres
    cube = UsdGeom.Cube.Define(stage, "/target_cube")
    cube.CreateSizeAttr(0.08)
    cube.AddTranslateOp().Set(Gf.Vec3d(*cube_center))
    cube.CreateDisplayColorAttr([Gf.Vec3f(1.0, 0.2, 0.1)])
    

    simulation_app.update()

    robot_path = articulation_path(robot_root)
    print(f"UR5e articulation: {robot_path}")

    commands_topic = args.joint_commands_topic.lstrip("/")
    states_topic = args.joint_states_topic.lstrip("/")

    og.Controller.edit(
        {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),

                # ORIGINAL (Isaac Sim 6.0):
                # The separate ReadJointState node is unavailable in Isaac Sim 5.1.
                # Keep the original line as a reference.
                # ("ReadJointState", "isaacsim.sensors.physics.IsaacReadJointState"),

                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],

            og.Controller.Keys.CONNECT: [

                # ORIGINAL (Isaac Sim 6.0):
                # The ReadJointState node separately reads the articulation and
                # supplies joint names, positions, velocities and efforts.
                # These connections cannot run in Isaac Sim 5.1.

                # ("OnPlaybackTick.outputs:tick", "ReadJointState.inputs:execIn"),
                # ("ReadJointState.outputs:execOut", "PublishJointState.inputs:execIn"),
                # ("ReadJointState.outputs:jointNames", "PublishJointState.inputs:jointNames"),
                # ("ReadJointState.outputs:jointPositions", "PublishJointState.inputs:jointPositions"),
                # ("ReadJointState.outputs:jointVelocities", "PublishJointState.inputs:jointVelocities"),
                # ("ReadJointState.outputs:jointEfforts", "PublishJointState.inputs:jointEfforts"),
                # ("ReadJointState.outputs:jointDofTypes", "PublishJointState.inputs:jointDofTypes"),
                # ("ReadJointState.outputs:stageMetersPerUnit", "PublishJointState.inputs:stageMetersPerUnit"),
                # ("ReadJointState.outputs:sensorTime", "PublishJointState.inputs:sensorTime"),

                # ISAAC SIM 5.1 REPLACEMENT:
                # Trigger the ROS joint-state publisher directly on each tick.
                # It reads joint states from the targetPrim specified below.
                ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),

                # Existing ROS 2 subscription and controller connections.
                ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),

                # ROS 2 contexts.
                ("Context.outputs:context", "PublishJointState.inputs:context"),
                ("Context.outputs:context", "SubscribeJointState.inputs:context"),
                ("Context.outputs:context", "PublishClock.inputs:context"),

                # ISAAC SIM 5.1 FIX:
                # Stamp published joint states using simulation time rather than zero.
                ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),

                # ORIGINAL — KEEP:
                # Continue publishing simulation time to /clock (unchanged).
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),

                # Forward ROS 2 joint commands to the robot articulation.
                ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                (
                    "SubscribeJointState.outputs:positionCommand",
                    "ArticulationController.inputs:positionCommand",
                ),
                (
                    "SubscribeJointState.outputs:velocityCommand",
                    "ArticulationController.inputs:velocityCommand",
                ),
                (
                    "SubscribeJointState.outputs:effortCommand",
                    "ArticulationController.inputs:effortCommand",
                ),
            ],

            og.Controller.Keys.SET_VALUES: [

                # The articulation controller operates the imported UR5e.
                ("ArticulationController.inputs:robotPath", robot_path),

                # ORIGINAL (Isaac Sim 6.0):
                # The ReadJointState sensor used to receive the articulation path.
                # ("ReadJointState.inputs:prim", [usdrt.Sdf.Path(robot_path)]),

                # ISAAC SIM 5.1 REPLACEMENT:
                # The ROS2PublishJointState node reads the articulation directly.
                # Therefore the robot path is assigned to its targetPrim input.
                ("PublishJointState.inputs:targetPrim", robot_path),

                # Preserve DataraAI's original ROS 2 topics.
                ("PublishJointState.inputs:topicName", states_topic),
                ("SubscribeJointState.inputs:topicName", commands_topic),
            ],
        },
    )

    # Optional settings: leave defaults if these attributes are unavailable.
    for attr_path, value in (
        ("/ActionGraph/Context.inputs:useDomainIDEnvVar", True),
        ("/ActionGraph/PublishClock.inputs:topicName", "clock"),
    ):
        try:
            og.Controller.set(og.Controller.attribute(attr_path), value)
        except Exception as exc:
            print(f"Left {attr_path} at its default: {exc}")

    simulation_app.update()

    # Problem 1 - Point 3: Publish the wrist RGB camera to ROS 2.
    # Keep the proven joint-control/clock ActionGraph above unchanged.
    # The render product captures 640x480 images from the tool0 camera.
    og.Controller.edit(
        {"graph_path": "/WristCameraGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("Tick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("Render", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("RGB", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                # Depth from the same wrist camera and render product.
                ("Depth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                # ROS 2 CameraInfo uses its own helper node in Isaac Sim 5.1.
                ("CameraInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            og.Controller.Keys.CONNECT: [
                ("Tick.outputs:tick", "Render.inputs:execIn"),
                ("Render.outputs:execOut", "RGB.inputs:execIn"),
                ("Render.outputs:renderProductPath", "RGB.inputs:renderProductPath"),
                ("Context.outputs:context", "RGB.inputs:context"),
                # Connect depth alongside RGB, preserving the working graph.
                ("Render.outputs:execOut", "Depth.inputs:execIn"),
                ("Render.outputs:renderProductPath", "Depth.inputs:renderProductPath"),
                ("Context.outputs:context", "Depth.inputs:context"),
                # Share the same render product and ROS 2 context with CameraInfo.
                ("Render.outputs:execOut", "CameraInfo.inputs:execIn"),
                ("Render.outputs:renderProductPath", "CameraInfo.inputs:renderProductPath"),
                ("Context.outputs:context", "CameraInfo.inputs:context"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("Render.inputs:cameraPrim", [usdrt.Sdf.Path(wrist_camera_path)]),
                ("Render.inputs:width", 640),
                ("Render.inputs:height", 480),
                ("RGB.inputs:type", "rgb"),
                ("RGB.inputs:topicName", "/wrist_cam/image_raw"),
                ("RGB.inputs:frameId", "camera_optical_frame"),
                ("RGB.inputs:useSystemTime", False),
                # At 60 simulation frames/s, publish once per four frames.
                ("RGB.inputs:frameSkipCount", 3),
                # Depth image stream (same frame and simulated timestamps).
                ("Depth.inputs:type", "depth"),
                ("Depth.inputs:topicName", "/wrist_cam/depth/image_raw"),
                ("Depth.inputs:frameId", "camera_optical_frame"),
                ("Depth.inputs:useSystemTime", False),
                ("Depth.inputs:frameSkipCount", 3),
                # Same optical frame, time base and simulation-frame rate as RGB.
                ("CameraInfo.inputs:topicName", "/wrist_cam/camera_info"),
                ("CameraInfo.inputs:frameId", "camera_optical_frame"),
                ("CameraInfo.inputs:useSystemTime", False),
                ("CameraInfo.inputs:frameSkipCount", 3),
            ],
        },
    )

    simulation_app.update()

    # ORIGINAL (Isaac Sim 6.0):
    # Sets the physics timestep and simulation device together.
    # SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")

    # ISAAC SIM 5.1 COMPATIBILITY:
    # Configure physics timestep and device using separate methods.
    SimulationManager.set_physics_dt(1.0 / 60.0)
    SimulationManager.set_physics_sim_device("cpu")

    # Isaac Sim 5.1: use the timeline to start physics playback.
    omni.timeline.get_timeline_interface().play()
    simulation_app.update()

    while simulation_app.is_running():
        simulation_app.update()

    # Stop simulation on exit.
    omni.timeline.get_timeline_interface().stop()
    simulation_app.close()


if __name__ == "__main__":
    main()
