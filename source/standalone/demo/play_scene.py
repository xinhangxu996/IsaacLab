# Copyright (c) 2022, NVIDIA CORPORATION & AFFILIATES, ETH Zurich, and University of Toronto
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
This script demonstrates how to use the scene interface to quickly setup a scene with multiple
articulated robots and sensors.
"""

from __future__ import annotations

"""Launch Isaac Sim Simulator first."""


import argparse

from omni.isaac.orbit.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="This script demonstrates how to use the scene interface.")
parser.add_argument("--num_envs", type=int, default=2, help="Number of environments to spawn.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""
import traceback

import carb

import omni.isaac.orbit.sim as sim_utils
from omni.isaac.orbit.assets import AssetBaseCfg
from omni.isaac.orbit.assets.config.anymal import ANYMAL_C_CFG
from omni.isaac.orbit.scene import InteractiveScene, InteractiveSceneCfg
from omni.isaac.orbit.sensors.ray_caster import RayCasterCfg, patterns
from omni.isaac.orbit.sim import SimulationContext
from omni.isaac.orbit.terrains import TerrainImporterCfg
from omni.isaac.orbit.utils import configclass
from omni.isaac.orbit.utils.timer import Timer


@configclass
class MySceneCfg(InteractiveSceneCfg):
    """Example scene configuration."""

    # terrain - flat terrain plane
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",
    )

    # articulation - robot 1
    robot_1 = ANYMAL_C_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot_1")
    # articulation - robot 2
    robot_2 = ANYMAL_C_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot_2")
    robot_2.init_state.pos = (0.0, 1.0, 0.6)

    # sensor - ray caster attached to the base of robot 1 that scans the ground
    height_scanner = RayCasterCfg(
        prim_path="{ENV_REGEX_NS}/Robot_1/base",
        offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 20.0)),
        attach_yaw_only=True,
        pattern_cfg=patterns.GridPatternCfg(resolution=0.1, size=[1.6, 1.0]),
        debug_vis=True,
        mesh_prim_paths=["/World/ground"],
    )

    # extras - light
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DistantLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75)),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, 500.0)),
    )


def main():
    """Main function."""

    # Load kit helper
    sim = SimulationContext(sim_utils.SimulationCfg(dt=0.005))
    # Set main camera
    sim.set_camera_view(eye=[5, 5, 5], target=[0.0, 0.0, 0.0])

    # Spawn things into stage
    with Timer("Setup scene"):
        scene = InteractiveScene(MySceneCfg(num_envs=args_cli.num_envs, env_spacing=5.0, lazy_sensor_update=False))

    # Play the simulator
    with Timer("Time taken to play the simulator"):
        sim.reset()

    # Now we are ready!
    print("[INFO]: Setup complete...")

    # default joint targets
    robot_1_actions = scene.articulations["robot_1"].data.default_joint_pos.clone()
    robot_2_actions = scene.articulations["robot_2"].data.default_joint_pos.clone()
    # Define simulation stepping
    sim_dt = sim.get_physics_dt()
    sim_time = 0.0
    count = 0
    # Simulate physics
    while simulation_app.is_running():
        # If simulation is stopped, then exit.
        if sim.is_stopped():
            break
        # If simulation is paused, then skip.
        if not sim.is_playing():
            sim.step(render=app_launcher.RENDER)
            continue
        # reset
        if count % 50 == 0:
            # reset counters
            sim_time = 0.0
            count = 0
            # reset root state
            root_state = scene.articulations["robot_1"].data.default_root_state_w.clone()
            root_state[:, :3] += scene.env_origins
            joint_pos = scene.articulations["robot_1"].data.default_joint_pos
            joint_vel = scene.articulations["robot_1"].data.default_joint_vel
            # -- set root state
            # -- robot 1
            scene.articulations["robot_1"].write_root_state_to_sim(root_state)
            scene.articulations["robot_1"].write_joint_state_to_sim(joint_pos, joint_vel)
            # -- robot 2
            root_state[:, 1] += 1.0
            scene.articulations["robot_2"].write_root_state_to_sim(root_state)
            scene.articulations["robot_2"].write_joint_state_to_sim(joint_pos, joint_vel)
            # reset buffers
            scene.reset()
            print(">>>>>>>> Reset!")
        # perform this loop at policy control freq (50 Hz)
        for _ in range(4):
            # set joint targets
            scene.articulations["robot_1"].set_joint_position_target(robot_1_actions)
            scene.articulations["robot_2"].set_joint_position_target(robot_2_actions)
            # write data to sim
            scene.write_data_to_sim()
            # perform step
            sim.step(render=app_launcher.RENDER)
            # read data from sim
            scene.update(sim_dt)
        # update sim-time
        sim_time += sim_dt * 4
        count += 1


if __name__ == "__main__":
    try:
        # run the main execution
        main()
    except Exception as err:
        carb.log_error(err)
        carb.log_error(traceback.format_exc())
        raise
    finally:
        # close sim app
        simulation_app.close()
