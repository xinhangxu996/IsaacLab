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

##
# Pre-defined configs
##
from omni.isaac.lab_assets import CARTPOLE_CFG  # isort:skip
import numpy as np

@configclass
class BouncerSceneCfg(InteractiveSceneCfg):
    """Configuration for a cart-pole scene."""

    # ground plane
    ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg())

    # lights
    dome_light = AssetBaseCfg(prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75)))

    # articulation
    cartpole: ArticulationCfg = CARTPOLE_CFG.replace(prim_path="/World/envs/env_.*/Robot")

    # Make balloon
    # replicate_physics = False
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

    balloon: RigidObjectCfg = RigidObjectCfg(
        prim_path="/World/envs/env_.*/Sphere",
        spawn=sim_utils.MultiAssetSpawnerCfg(
            assets_cfg=[
                sim_utils.SphereCfg(
                    radius=0.5,
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0), metallic=0.2),
                ),
            ],
            random_choice=True,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                linear_damping=10.0
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.01),
            collision_props=sim_utils.CollisionPropertiesCfg(),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.0, 0.0, 5.0)),
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
    scene_cfg = BouncerSceneCfg(num_envs=2, env_spacing=2.0)
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
