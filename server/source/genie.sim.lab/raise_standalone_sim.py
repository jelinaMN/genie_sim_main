# Copyright (c) 2023-2025, AgiBot Inc. All Rights Reserved.
# Author: Genie Sim Team
# License: Mozilla Public License Version 2.0

import argparse
import os
from pathlib import Path
import sys

# 添加项目根目录到系统路径（确保内部模块可导入）
project_root = Path(__file__).resolve().parent.parent.parent.parent
sys.path.append(str(project_root))

# 定义命令行参数
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
#检查和修复当前 Python 运行环境
base_utils.check_and_fix_env()

from genie.sim.lab.app import AppLauncher

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)

# 解析参数并设置环境变量
args_cli = parser.parse_args()
os.environ["ISAACSIM_VERSION"] = args_cli.isaacsim_version


# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app


import omni
# 根据 Isaac Sim 版本导入不同的 World 类
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
        stage_units_in_meters=1, # 单位设置为米
        physics_dt=physics_dt,   # 物理时间步长（秒）
        rendering_dt=1.0 / args_cli.rendering_step, # 渲染时间步长（秒）
    )
    # Override CPU setting to use GPU
    physx_interface = omni.physx.get_physx_interface()
    physx_interface.overwrite_gpu_setting(1)
    world._physics_context.enable_gpu_dynamics(flag=True)
    # 启用连续碰撞检测
    world._physics_context.enable_ccd(flag=True) 
    # 构建用户界面
    ui_builder = UIBuilder(world=world)
    #CommandController: 处理仿真逻辑（如机器人控制、传感器数据处理、ROS通信等）。
    server_function = CommandController(
        ui_builder=ui_builder,
        enable_physics=not args_cli.disable_physics,
        enable_curobo=args_cli.enable_curobo,
        rendering_step=args_cli.rendering_step,
        publish_ros=args_cli.rospub,
        record_images=args_cli.record_img,
    )
    # 启动 gRPC 服务器，允许远程控制
    rpc_server = GrpcServer(server_function=server_function)
    rpc_server.start()

    step = 0
    while simulation_app.is_running():
        step += 1
        # 更新物理世界和渲染
        ui_builder.my_world.step(render=True)
        if rpc_server:
            rpc_server.server_function.on_physics_step()
            if rpc_server.server_function.exit:
                break
    # 关闭应用
    # simulation_app.close()


if __name__ == "__main__":
    # run the main execution
    main()
