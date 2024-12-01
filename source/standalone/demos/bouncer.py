# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""This script demonstrates how to use the interactive scene interface to setup a scene with multiple prims.

.. code-block:: bash

    # Usage
    ./isaaclab.sh -p source/standalone/tutorials/02_scene/create_scene.py --num_envs 32

"""

"""Launch Isaac Sim Simulator first."""


import argparse

from omni.isaac.lab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Tutorial on using the interactive scene interface.")
parser.add_argument("--num_envs", type=int, default=2, help="Number of environments to spawn.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import torch

import omni.isaac.lab.sim as sim_utils
import omni.isaac.lab.utils.math as math_utils
from omni.isaac.lab.assets import ArticulationCfg, AssetBaseCfg, RigidObject, RigidObjectCfg, DeformableObject, DeformableObjectCfg
from omni.isaac.lab.scene import InteractiveScene, InteractiveSceneCfg
from omni.isaac.lab.sim import SimulationContext
from omni.isaac.lab.utils import configclass
from omni.isaac.core.objects import FixedCuboid,DynamicSphere,VisualCuboid

##
# Pre-defined configs
##
# from omni.isaac.lab_assets import CARTPOLE_CFG  # isort:skip

from omni.isaac.lab_assets.unitree import UNITREE_GO1_CFG  # isort: skip
from omni.isaac.lab.sensors import CameraCfg, ContactSensorCfg, RayCasterCfg, patterns
import omni.isaac.core.utils.prims as prim_utils

import numpy as np

wall_h=10.0
wall_w=6.0

@configclass
class BouncerSceneCfg(InteractiveSceneCfg):
    """Configuration for a cart-pole scene."""

    # ground plane
    ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg())

    # lights
    dome_light = AssetBaseCfg(prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75)))

    # articulation
    # cartpole: ArticulationCfg = CARTPOLE_CFG.replace(prim_path="/World/envs/env_.*/Robot")
    cartpole: ArticulationCfg = UNITREE_GO1_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    camera = CameraCfg(

        prim_path="{ENV_REGEX_NS}/Robot/trunk/camera_face/camera",

        update_period=0.1,

        height=480,

        width=640,

        data_types=["rgb", "distance_to_image_plane"],

        spawn=sim_utils.PinholeCameraCfg(

            focal_length=24.0, focus_distance=400.0, horizontal_aperture=20.955, clipping_range=(0.1, 1.0e5)

        ),

        offset=CameraCfg.OffsetCfg(pos=(0.510, 0.0, 0.015), rot=(0.5, -0.5, 0.5, -0.5), convention="ros"),

    )
    # VisualCuboid(prim_path="/World/envs/env_0/ball", translation=np.array([0.0, 0.0, 5.0]),size=10.0)

    # prim_utils.define_prim("/World/envs/env_*")


    # FixedCuboid(prim_path="{ENV_REGEX_NS}/wall1", translation=np.array([3.1, 0.0, 5.0]), size=np.array([0.2, 6, 20]))
    # height_scanner = RayCasterCfg(

    #     prim_path="{ENV_REGEX_NS}/Robot/base",

    #     update_period=0.02,

    #     offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 20.0)),

    #     attach_yaw_only=True,

    #     pattern_cfg=patterns.GridPatternCfg(resolution=0.1, size=[1.6, 1.0]),

    #     debug_vis=True,

    #     mesh_prim_paths=["/World/defaultGroundPlane"],

    # )

    # contact_forces = ContactSensorCfg(

    #     prim_path="{ENV_REGEX_NS}/Robot/.*_FOOT", update_period=0.0, history_length=6, debug_vis=True

    # )

    # Make balloon
    replicate_physics = False
    # balloon: DeformableObjectCfg = DeformableObjectCfg(
    #     prim_path="/World/envs/env_.*/Sphere",
    #     spawn=sim_utils.MultiAssetSpawnerCfg(
    #         assets_cfg=[
    #             sim_utils.MeshSphereCfg(
    #                 radius=0.5,
    #                 deformable_props=sim_utils.DeformableBodyPropertiesCfg(rest_offset=0.0, contact_offset=0.001),
    #                 visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.5, 0.1, 0.0)),
    #                 physics_material=sim_utils.DeformableBodyMaterialCfg(poissons_ratio=0.4, youngs_modulus=1e5),
    #             ),
    #         ],
    #         random_choice=False,
    #         deformable_props=sim_utils.DeformableBodyPropertiesCfg(
    #             # rest_offset=0.0, contact_offset=0.001,
    #             solver_position_iteration_count=4, vertex_velocity_damping=5
    #         ),
    #         mass_props=sim_utils.MassPropertiesCfg(mass=0.01),
    #         # collision_props=sim_utils.CollisionPropertiesCfg(),
    #     ),
    #     init_state=DeformableObjectCfg.InitialStateCfg(pos=[0, 0, 7]),
    # )



    # wall_r:RigidObject=RigidObjectCfg(
    #     prim_path="/World/envs/env_.*/Wall_r",
    #     spawn=sim_utils.MultiAssetSpawnerCfg(
    #         assets_cfg=[
    #             sim_utils.CuboidCfg(
    #                 size=(wall_w, 6, wall_h),
    #                 visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 1.0, 1.0), metallic=0.2),
    #             )
    #         ],
    #         random_choice=True,
    #         rigid_props=sim_utils.RigidBodyPropertiesCfg(
    #             linear_damping=5.0
    #         ),
    #         mass_props=sim_utils.MassPropertiesCfg(mass=10000),
    #         collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
    #     ),
    #     init_state=RigidObjectCfg.InitialStateCfg(pos=(wall_w*0.5+3, 0.0, 5.0)),
    # )

    # wall_l:RigidObject=RigidObjectCfg(
    #     prim_path="/World/envs/env_.*/Wall_l",
    #     spawn=sim_utils.MultiAssetSpawnerCfg(
    #         assets_cfg=[
    #             sim_utils.CuboidCfg(
    #                 size=(wall_w, 6, wall_h),
    #                 visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 1.0, 1.0), metallic=0.2),
    #             )
    #         ],
    #         random_choice=True,
    #         rigid_props=sim_utils.RigidBodyPropertiesCfg(
    #             linear_damping=5.0
    #         ),
    #         mass_props=sim_utils.MassPropertiesCfg(mass=10000),
    #         collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
    #     ),
    #     init_state=RigidObjectCfg.InitialStateCfg(pos=(-wall_w*0.5-3, 0.0, 5.0)),
    # )

    # wall_f:RigidObject=RigidObjectCfg(
    #     prim_path="/World/envs/env_.*/Wall_f",
    #     spawn=sim_utils.MultiAssetSpawnerCfg(
    #         assets_cfg=[
    #             sim_utils.CuboidCfg(
    #                 size=(6, wall_w, wall_h),
    #                 visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 1.0, 1.0), metallic=0.2),
    #             )
    #         ],
    #         random_choice=True,
    #         rigid_props=sim_utils.RigidBodyPropertiesCfg(
    #             linear_damping=5.0
    #         ),
    #         mass_props=sim_utils.MassPropertiesCfg(mass=10000),
    #         collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
    #     ),
    #     init_state=RigidObjectCfg.InitialStateCfg(pos=( 0.0,wall_w*0.5+3, 5.0)),
    # )

    # wall_b:RigidObject=RigidObjectCfg(
    #     prim_path="/World/envs/env_.*/Wall_b",
    #     spawn=sim_utils.MultiAssetSpawnerCfg(
    #         assets_cfg=[
    #             sim_utils.CuboidCfg(
    #                 size=(6, wall_w, wall_h),
    #                 visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 1.0, 1.0), metallic=0.2),
    #             )
    #         ],
    #         random_choice=True,
    #         rigid_props=sim_utils.RigidBodyPropertiesCfg(
    #             linear_damping=5.0
    #         ),
    #         mass_props=sim_utils.MassPropertiesCfg(mass=10000.0),
    #         collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
    #     ),
    #     init_state=RigidObjectCfg.InitialStateCfg(pos=( 0.0,-wall_w*0.5-3, 5.0)),
    # ) # type: ignore

    balloon: RigidObjectCfg = RigidObjectCfg(
        prim_path="/World/envs/env_.*/Sphere",
        spawn=sim_utils.MultiAssetSpawnerCfg(
            assets_cfg=[
                sim_utils.SphereCfg(
                    radius=0.5,
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0), metallic=0.2),
                ),
                sim_utils.SphereCfg(
                    radius=0.5,
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 1.0, 0.0), metallic=0.2),
                ),
                sim_utils.SphereCfg(
                    radius=0.5,
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 0.0, 1.0), metallic=0.2),
                ),
            ],
            random_choice=True,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                linear_damping=5.0
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.01),
            collision_props=sim_utils.CollisionPropertiesCfg(),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(2.0, 0.0, 5.0)),
    )




def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    """Runs the simulation loop."""
    # Extract scene entities
    # note: we only do this here for readability.
    robot = scene["cartpole"]
    balloon = scene["balloon"]
    # Define simulation stepping
    sim_dt = sim.get_physics_dt()
    count = 0
    # Simulation loop
    


    while simulation_app.is_running():
        # Reset
        if count % 1250 == 0:
            # reset counter
            count = 0
            # reset the scene entities
            # root state
            # we offset the root state by the origin since the states are written in simulation world frame
            # if this is not done, then the robots will be spawned at the (0, 0, 0) of the simulation world
            root_state = robot.data.default_root_state.clone()
            root_state[:, :3] += scene.env_origins
            robot.write_root_state_to_sim(root_state)
            # set joint positions with some noise
            joint_pos, joint_vel = robot.data.default_joint_pos.clone(), robot.data.default_joint_vel.clone()
            joint_pos += torch.rand_like(joint_pos) * 0.1
            robot.write_joint_state_to_sim(joint_pos, joint_vel)



            print(scene.rigid_objects.keys)

            if isinstance(balloon, DeformableObject):
                
                # Reset the balloons
                balloon_state = balloon.data.default_nodal_state_w.clone()
                balloon.write_nodal_state_to_sim(balloon_state)
                balloon.reset()

            elif isinstance(balloon, RigidObject):
                
                print(balloon.body_names)

                # Reset the balloons
                balloon_state = balloon.data.default_root_state.clone()
                balloon_state[:, :3] += scene.env_origins
                balloon.write_root_state_to_sim(balloon_state)
                balloon.reset()

            # clear internal buffers
            scene.reset()
            print("[INFO]: Resetting robot state...")

        # Apply random action
        # -- generate random joint efforts
        efforts = torch.randn_like(robot.data.joint_pos) * 5.0
        # -- apply action to the robot
        robot.set_joint_effort_target(efforts)
        # -- write data to sim
        scene.write_data_to_sim()
        # Perform step
        sim.step()
        # Increment counter
        count += 1
        # Update buffers
        scene.update(sim_dt)
        print("-------------------------------")

        print(scene["camera"])

        print("Received shape of rgb   image: ", scene["camera"].data.output["rgb"].shape)

        print("Received shape of depth image: ", scene["camera"].data.output["distance_to_image_plane"].shape)

        print("-------------------------------")

        # print(scene["height_scanner"])

        # print("Received max height value: ", torch.max(scene["height_scanner"].data.ray_hits_w[..., -1]).item())

        # print("-------------------------------")

        # print(scene["contact_forces"])

        # print("Received max contact force of: ", torch.max(scene["contact_forces"].data.net_forces_w).item())




def findCamTarget(vp, rz, rx):
    
    x = vp[0]
    y = vp[1]
    z = vp[2]

    pi  = np.pi
    cos = np.cos
    sin = np.sin

    rx = rx*pi/180
    rz = rz*pi/180

    t = [0, 0, 0]

    if abs(cos(rx) > 1e-3):    # There can be an intersection with ground plane z = 0
        print("Intersection with ground")
        k = z/cos(rx)
        t = [x - k*sin(rz)*sin(rx), y + k*cos(rz)*sin(rx), z - k*cos(rx)]
    elif abs(cos(rz)) > 1e-3:  # There can be an intersection with ground plane y = 0
        print("Intersection with y = 0")
        k = -y/cos(rz)
        t = [x - k*sin(rz), 0, z]
    else:  # There can be an intersection with ground plane x = 0
        print("Intersection with x = 0")
        k = x/sin(rz)
        t = [0, y + k*cos(rz), z]
    
    return t

def main():

    """Main function."""
    # Load kit helper
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = SimulationContext(sim_cfg)
    
    # Set the view
    campos = [-25, -7, 9]
    camtar = findCamTarget(vp=campos, rz=-72, rx=72)
    sim.set_camera_view(campos, camtar)

    # Design scene
# scene_cfg = BouncerSceneCfg(num_envs=args_cli.num_envs, env_spacing=2*wall_w+1)
    scene_cfg = BouncerSceneCfg(num_envs=args_cli.num_envs, env_spacing=50)
    scene = InteractiveScene(scene_cfg)

    # Play the simulator
    sim.reset()
    
    # Now we are ready!
    print("[INFO]: Setup complete...")
    
    # Run the simulator
    run_simulator(sim, scene)


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
