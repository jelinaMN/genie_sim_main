# Copyright (c) 2023-2025, AgiBot Inc. All Rights Reserved.
# Author: Genie Sim Team
# License: Mozilla Public License Version 2.0

import argparse
import os
from pathlib import Path
import sys

project_root = Path(__file__).resolve().parent.parent.parent.parent
sys.path.append(str(project_root))

# add argparse arguments
parser = argparse.ArgumentParser(description="standalone_sim sever launcher script.")
parser.add_argument("--physics_step", type=int, default=120)
parser.add_argument("--isaacsim_version", type=str, default="v45")
parser.add_argument("--rendering_step", type=int, default=30)
parser.add_argument("--enable_curobo", type=bool, default=False)
parser.add_argument("--rospub", action="store_true", help="start rospub")
parser.add_argument("--record_img", action="store_true", help="start recording images")
parser.add_argument("--render_mode", type=str, default="RaytracedLighting")
parser.add_argument(
    "--disable_physics", action="store_true", default=False, help="disable physics"
)

import base_utils

base_utils.check_and_fix_env()

from genie.sim.lab.app import AppLauncher

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)

# parse the arguments
args_cli = parser.parse_args()
os.environ["ISAACSIM_VERSION"] = args_cli.isaacsim_version


# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app


import omni

if os.getenv("ISAACSIM_VERSION") == "v45":
    from isaacsim.core.api import World
else:
    from omni.isaac.core import World

from genie.sim.lab.controllers import CommandController
from robot.isaac_sim.grpc_server import GrpcServer
from genie.sim.lab.app.ui_builder import UIBuilder

if args_cli.rospub:
    if os.getenv("ISAACSIM_VERSION") == "v45":
        from isaacsim.core.utils import extensions

        extensions.enable_extension("isaacsim.ros2.bridge")
    else:
        from omni.isaac.core.utils import extensions

        extensions.enable_extension("omni.isaac.ros2_bridge")


def main():
    """Main function."""

    physics_dt = 1.0 / args_cli.physics_step
    world = World(
        stage_units_in_meters=1,
        physics_dt=physics_dt,
        rendering_dt=1.0 / args_cli.rendering_step,
    )
    # Override CPU setting to use GPU
    physx_interface = omni.physx.get_physx_interface()
    physx_interface.overwrite_gpu_setting(1)
    world._physics_context.enable_gpu_dynamics(flag=True)
    world._physics_context.enable_ccd(flag=True)
    ui_builder = UIBuilder(world=world)
    server_function = CommandController(
        ui_builder=ui_builder,
        enable_physics=not args_cli.disable_physics,
        enable_curobo=args_cli.enable_curobo,
        rendering_step=args_cli.rendering_step,
        publish_ros=args_cli.rospub,
        record_images=args_cli.record_img,
    )
    rpc_server = GrpcServer(server_function=server_function)
    rpc_server.start()

    step = 0
    while simulation_app.is_running():
        step += 1

        ui_builder.my_world.step(render=True)
        if rpc_server:
            rpc_server.server_function.on_physics_step()
            if rpc_server.server_function.exit:
                break

    simulation_app.close()


if __name__ == "__main__":
    # run the main execution
    main()
