# Copyright (c) 2023-2025, AgiBot Inc. All Rights Reserved.
# Author: Genie Sim Team
# License: Mozilla Public License Version 2.0

# Send command
# 001. Take a photo
# 002. Left hand/right hand moves the specified position
# 003. Move the whole body joints to a specified angle
# 004. Get gripper pose
# 005. Get object pose
# 006. Add object
import numpy as np
import os, time

from scipy.spatial.transform import Rotation
from robot import Robot
from robot.isaac_sim.client import Rpc_Client
from robot.utils import (
    get_quaternion_wxyz_from_rotation_matrix,
    get_rotation_matrix_from_quaternion,
    matrix_to_euler_angles,
    get_quaternion_from_euler,
    get_quaternion_from_rotation_matrix,
    quat_multiplication,
    quat_to_rot_matrix,
    get_rotation_matrix_from_euler,
)


from base_utils.logger import Logger

logger = Logger()  # Create singleton instance

import ik_solver
from copy import deepcopy

SIM_REPO_ROOT = os.getenv("SIM_REPO_ROOT")


def mat_to_rot6d(mat):
    batch_dim = mat.shape[:-2]
    out = mat[..., :2, :].copy().reshape(batch_dim + (6,))
    return out


def normalize(vec, eps=1e-12):
    norm = np.linalg.norm(vec, axis=-1)
    norm = np.maximum(norm, eps)
    out = (vec.T / norm).T
    return out


def rot6d_to_mat(d6):
    a1, a2 = d6[..., :3], d6[..., 3:]
    b1 = normalize(a1)
    b2 = a2 - np.sum(b1 * a2, axis=-1, keepdims=True) * b1
    b2 = normalize(b2)
    b3 = np.cross(b1, b2, axis=-1)
    out = np.stack((b1, b2, b3), axis=-2)
    return out


def pose10d_to_mat(d10):
    pos = d10[..., :3]
    d6 = d10[..., 3:]
    rotmat = rot6d_to_mat(d6)
    out = np.zeros(d10.shape[:-1] + (4, 4), dtype=d10.dtype)
    out[..., :3, :3] = rotmat
    out[..., :3, 3] = pos
    out[..., 3, 3] = 1
    return out


def get_transform(trans, rot):
    mat = np.zeros((4, 4))
    mat[:3, :3] = rot
    mat[:3, 3] = trans
    mat[3, 3] = 1
    return mat


class IsaacSimRpcRobot(Robot):
    def __init__(
        self,
        robot_cfg="G1_120s.json",
        scene_usd="Pick_Place_G1_Yellow_Table.usd",
        client_host="localhost:50051",
        position=[0, 0, 0],
        rotation=[1, 0, 0, 0],
        gripper_control_type=0,
    ):
        robot_urdf = robot_cfg.split(".")[0] + ".urdf"
        self.client = Rpc_Client(client_host, robot_urdf)
        self.client.InitRobot(
            robot_cfg=robot_cfg,
            robot_usd="",
            scene_usd=scene_usd,
            init_position=position,
            init_rotation=rotation,
        )
        self.joint_names = {
            "left": [
                "Joint1_l",
                "Joint2_l",
                "Joint3_l",
                "Joint4_l",
                "Joint5_l",
                "Joint6_l",
                "Joint7_l",
            ],
            "right": [
                "Joint1_r",
                "Joint2_r",
                "Joint3_r",
                "Joint4_r",
                "Joint5_r",
                "Joint6_r",
                "Joint7_r",
            ],
        }
        self.cam_info = None
        self.robot_gripper_2_grasp_gripper = np.array(
            [[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]]
        )
        self.init_position = position
        self.init_rotation = rotation
        self.gripper_control_type = gripper_control_type
        self.last_eef_pos = [(), ()]
        self.init_transform()
        self.setup()

    def reset(self):
        self.target_object = None
        self.client.reset()
        time.sleep(0.5)

        self.close_gripper(id="right")
        time.sleep(0.5)
        pass

    def setup(self):
        self.target_object = None
        self.ik_solver = ik_solver.Solver(
            urdf_path=f"{SIM_REPO_ROOT}/base_utils/IK-SDK/G1.urdf",
            config_path=str(
                os.path.join(f"{SIM_REPO_ROOT}/base_utils/IK-SDK/config", "solver.yaml")
            ),
            use_relaxed_ik=True,
            use_elbow=False,
        )

        # set robot init state

    def get_observation(self, data_keys):
        """
        Example
            data_keys = {
                'camera': {
                    'camera_prim_list': [
                        '/World/G1/head_link/D455_Solid/TestCameraDepth'
                    ],
                    'render_depth': True,
                    'render_semantic': True
                },
                'pose': [
                    '/World/G1/head_link/D455_Solid/TestCameraDepth'
                ],
                'joint_position': True,
                'gripper': True
            }
        """

        observation = {}
        observation = self.client.get_observation(data_keys)

        if "camera" in data_keys:
            render_depth = (
                data_keys["camera"]["render_depth"]
                if "render_depth" in data_keys["camera"]
                else False
            )
            render_semantic = (
                data_keys["camera"]["render_semantic"]
                if "render_semantic" in data_keys["camera"]
                else False
            )

            cam_data = {}
            for cam_prim in data_keys["camera"]["camera_prim_list"]:
                cam_data[cam_prim] = {}
                response = self.client.capture_frame(camera_prim_path=cam_prim)
                # cam_info
                cam_info = {
                    "W": response.color_info.width,
                    "H": response.color_info.height,
                    "K": np.array(
                        [
                            [response.color_info.fx, 0, response.color_info.ppx],
                            [0, response.color_info.fy, response.color_info.ppy],
                            [0, 0, 1],
                        ]
                    ),
                    "scale": 1,
                }
                cam_data[cam_prim]["cam_info"] = cam_info
                # c2w
                c2w = self.get_prim_world_pose(cam_prim, camera=True)
                cam_data[cam_prim]["c2w"] = c2w
                # rgb
                rgb = np.frombuffer(response.color_image.data, dtype=np.uint8).reshape(
                    cam_info["H"], cam_info["W"], 4
                )[:, :, :3]
                cam_data[cam_prim]["image"] = rgb
                # depth
                if render_depth:
                    depth = np.frombuffer(
                        response.depth_image.data, dtype=np.float32
                    ).reshape(cam_info["H"], cam_info["W"])
                    cam_data[cam_prim]["depth"] = depth

                # semantic
                if render_semantic:
                    response = self.client.capture_semantic_frame(
                        camera_prim_path=cam_prim
                    )
                    prim_id = {}
                    for label in response.label_dict:
                        name, id = label.label_name, label.label_id
                        prim = "/World/Objects/" + name
                        prim_id[prim] = id
                    mask = np.frombuffer(
                        response.semantic_mask.data, dtype=np.int32
                    ).reshape(cam_info["H"], cam_info["W"])
                    cam_data[cam_prim]["semantic"] = {"prim_id": prim_id, "mask": mask}

            observation["camera"] = cam_data

        if "pose" in data_keys:
            pose_data = {}
            for obj_prim in data_keys["pose"]:
                pose_data[obj_prim] = self.get_prim_world_pose(prim_path=obj_prim)
            observation["pose"] = pose_data

        if "joint_position" in data_keys:
            joint_position = self.client.get_joint_positions()
            observation["joint_position"] = joint_position

        if "gripper" in data_keys:
            gripper_state = {}
            gripper_state["left"] = self.client.get_gripper_state(is_right=False)
            gripper_state["right"] = self.client.get_gripper_state(is_right=True)
            observation["gripper"] = gripper_state

        return observation

    def open_gripper(self, id="left", width=0.5):
        is_Right = True if id == "right" else False
        # if width is None:
        width = 0.08
        self.client.set_gripper_state(
            gripper_command="open", is_right=is_Right, opened_width=width
        )
        self.client.DetachObj()

    def close_gripper(self, id="left", force=150):
        is_Right = True if id == "right" else False
        self.client.set_gripper_state(
            gripper_command="close", is_right=is_Right, opened_width=0.00
        )
        if self.target_object is not None and is_Right:
            self.client.DetachObj()
            self.client.AttachObj(prim_paths=["/World/Objects/" + self.target_object])

    def move_pose(self, target_pose, type, arm="right", **kwargs):
        if type.lower() == "trajectory":
            content = {
                "type": "trajectory_4x4_pose",
                "data": target_pose,
            }
        else:
            if type == "AvoidObs":
                type = "ObsAvoid"
            elif type == "Normal":
                type = "Simple"

            content = {
                "type": "matrix",
                "matrix": target_pose,
                "trajectory_type": type,
                "arm": arm,
            }
        return self.move(content)

    def set_gripper_action(self, action, arm="right"):
        # self.gripper_server.set_gripper_action(action)
        assert arm in ["left", "right"]
        time.sleep(0.3)
        if action is None:
            return
        if action == "open":
            self.open_gripper(id=arm, width=0.1)
        elif action == "close":
            self.close_gripper(id=arm)

    def move(self, content):
        """
        type: str, 'matrix' or 'joint'
            'pose': np.array, 4x4
            'joint': np.array, 1xN
        """
        type = content["type"]
        arm_name = content.get("arm", "right")
        if type == "matrix":
            if isinstance(content["matrix"], list):
                content["matrix"] = np.array(content["matrix"])
            R, T = content["matrix"][:3, :3], content["matrix"][:3, 3]
            quat_wxyz = get_quaternion_from_euler(
                matrix_to_euler_angles(R), order="ZYX"
            )
            ee_interpolation = False
            target_position = T
            if "trajectory_type" in content and content["trajectory_type"] == "Simple":
                is_backend = True
                target_rotation = quat_wxyz
            elif (
                "trajectory_type" in content
                and content["trajectory_type"] == "Straight"
            ):
                is_backend = True
                target_rotation = quat_wxyz
                ee_interpolation = True
            else:
                is_backend = False
                init_rotation_matrix = get_rotation_matrix_from_quaternion(
                    self.init_rotation
                )
                translation_matrix = np.zeros((4, 4))
                translation_matrix[:3, :3] = init_rotation_matrix
                translation_matrix[:3, 3] = self.init_position
                translation_matrix[3, 3] = 1
                target_matrix = np.linalg.inv(translation_matrix) @ content["matrix"]
                target_rotation_matrix, target_position = (
                    target_matrix[:3, :3],
                    target_matrix[:3, 3],
                )
                target_rotation = get_quaternion_from_euler(
                    matrix_to_euler_angles(target_rotation_matrix), order="ZYX"
                )

                logger.info(f"target_position is{target_position}")
                logger.info(f"target_rotation is{target_rotation}")

            state = (
                self.client.moveto(
                    target_position=target_position,
                    target_quaternion=target_rotation,
                    arm_name=arm_name,
                    is_backend=is_backend,
                    ee_interpolation=ee_interpolation,
                ).errmsg
                == "True"
            )
            logger.info(f"move! {arm_name} {T} {quat_wxyz} {state}")

        elif type == "quat":

            if "trajectory_type" in content and content["trajectory_type"] == "Simple":
                is_backend = True
            else:
                is_backend = False
            state = (
                self.client.moveto(
                    target_position=content["xyz"],
                    target_quaternion=content["quat"],
                    arm_name="left",
                    is_backend=is_backend,
                ).errmsg
                == "True"
            )

        elif type == "joint":
            gripper_control_type = 0  # 0 Position mode, 1 Force mode
            if gripper_control_type == 0:
                joint_indices = np.arange(18)
                joint_indices = np.append(joint_indices, [18, 20])
                state = self.client.set_joint_positions(
                    content["position"], content["is_trajectory"], joint_indices
                )
            else:
                left_joint = content["position"][18]
                right_joint = content["position"][19]
                if left_joint < 0.5:
                    self.client.set_gripper_state(
                        gripper_command="close", is_right=False, opened_width=0.00
                    )
                else:
                    self.client.set_gripper_state(
                        gripper_command="open", is_right=False, opened_width=0.08
                    )

                if right_joint < 0.5:
                    self.client.set_gripper_state(
                        gripper_command="close", is_right=True, opened_width=0.00
                    )
                else:
                    self.client.set_gripper_state(
                        gripper_command="open", is_right=True, opened_width=0.08
                    )

                main_joint_position = content["position"][:18]
                joint_indices = np.arange(18)
                state = self.client.set_joint_positions(
                    main_joint_position, content["is_trajectory"], joint_indices
                )

        elif type == "euler":
            is_backend = True

            T_curr = self.get_ee_pose()
            xyzrpy_input = content["xyzrpy"]
            xyz_curr = T_curr[:3, 3]
            rpy_curr = Rotation.from_matrix(T_curr[:3, :3]).as_euler("xyz")

            incr = content.get("incr", False)
            if incr:
                xyz_tgt = xyz_curr + np.array(xyzrpy_input[:3])
                rpy_tgt = rpy_curr + np.array(xyzrpy_input[3:])
                quat_tgt = get_quaternion_from_rotation_matrix(
                    Rotation.from_euler("xyz", rpy_tgt).as_matrix()
                )
            else:
                raise NotImplementedError

            state = (
                self.client.moveto(
                    target_position=xyz_tgt,
                    target_quaternion=quat_tgt,
                    arm_name="left",
                    is_backend=is_backend,
                ).errmsg
                == "True"
            )

        elif type == "move_pose_list":
            action_list = content["data"]
            for action in action_list:
                T_curr = content["ee_transform"]
                action_pose10d = action[:-1]
                action_gripper = action[-1]
                xyz_curr = T_curr[:3, 3]
                rotation_curr = T_curr[:3, :3]

                action_repr = content.get("action_repr", "rela")
                if action_repr == "delta":
                    mat_tgt = pose10d_to_mat(action_pose10d)
                    xyz_delta = mat_tgt[:3, 3]
                    rotation_delta = mat_tgt[:3, :3]
                    xyz_tgt = xyz_curr + np.array(xyz_delta[:3])
                    rotation_tgt = np.dot(rotation_curr, rotation_delta)
                    quat_tgt = get_quaternion_from_rotation_matrix(rotation_tgt)
                else:
                    raise RuntimeError()

                start_time = time.time()
                state = (
                    self.client.moveto(
                        target_position=xyz_tgt,
                        target_quaternion=quat_tgt,
                        arm_name="left",
                        is_backend=True,
                    ).errmsg
                    == "True"
                )
                xyz_curr = self.get_ee_pose()[:3, 3]
                euler_curr = Rotation.from_matrix(self.get_ee_pose()[:3, :3]).as_euler(
                    "xyz"
                )
                euler_tgt = Rotation.from_matrix(rotation_tgt).as_euler("xyz")

                gripper_position = content["gripper_position"]
                start_time = time.time()
                if gripper_position > 0.03 and action_gripper < 0.5:
                    self.close_gripper(id="right", force=50)
                    logger.info(f"Close gripper {time.time() - start_time}")
                elif gripper_position < 0.03 and action_gripper > 0.5:
                    self.open_gripper(id="right", width=0.1)
                    logger.info(f"Open gripper {time.time() - start_time}")

        elif type.lower() == "trajectory":
            action_list = content["data"]

            T_curr = self.get_ee_pose()
            xyz_curr = T_curr[:3, 3]
            rotation_curr = T_curr[:3, :3]

            traj = []
            for action in action_list:
                xyzrpy_input = action[:3]

                action_repr = content.get("action_repr", "rela")
                if action_repr == "delta":
                    xyz_tgt = xyz_curr + np.array(xyzrpy_input[:3])
                    xyz_curr = xyz_tgt.copy()
                elif action_repr == "abs":
                    xyz_tgt = np.array(xyzrpy_input[:3])

                action_rotation_matrix = Rotation.from_euler(
                    "xyz", np.array([np.pi, 0, np.pi])
                ).as_matrix()
                quat_tgt = Rotation.from_matrix(action_rotation_matrix).as_quat()

                pose = list(xyz_tgt), list(quat_tgt)
                traj.append(pose)

            # logger.debug("traj", traj)
            start_time = time.time()
            self.client.SetTrajectoryList(traj)

            xyz_curr = self.get_ee_pose()[:3, 3]
            # logger.debug(xyz_curr, xyz_tgt)
            logger.info("xyz dist {np.linalg.norm(xyz_curr - xyz_tgt)}")
            logger.info("move", time.time() - start_time)

            # time.sleep(100)

        elif type.lower() == "trajectory_4x4_pose":
            waypoint_list = content["data"]

            traj = []
            for pose in waypoint_list:
                xyz = pose[:3, 3]
                quat_wxyz = get_quaternion_from_rotation_matrix(pose[:3, :3])
                pose = list(xyz), list(quat_wxyz)
                traj.append(pose)

            logger.info("Set Trajectory!")
            self.client.SetTrajectoryList(traj, is_block=True)

            # self.client.SetTrajectoryList(traj)           # err: not block
            state = True

        else:
            raise NotImplementedError

        return state

    def get_prim_world_pose(self, prim_path, camera=False):
        rotation_x_180 = np.array(
            [
                [1.0, 0.0, 0.0, 0],
                [0.0, -1.0, 0.0, 0],
                [0.0, 0.0, -1.0, 0],
                [0, 0, 0, 1],
            ],
        )
        response = self.client.get_object_pose(prim_path=prim_path)
        x, y, z = (
            response.object_pose.position.x,
            response.object_pose.position.y,
            response.object_pose.position.z,
        )
        quat_wxyz = np.array(
            [
                response.object_pose.rpy.rw,
                response.object_pose.rpy.rx,
                response.object_pose.rpy.ry,
                response.object_pose.rpy.rz,
            ]
        )
        rot_mat = get_rotation_matrix_from_quaternion(quat_wxyz)

        pose = np.eye(4)
        pose[:3, :3] = rot_mat
        pose[:3, 3] = np.array([x, y, z])

        if camera:
            pose = pose @ rotation_x_180
        return pose

    def pose_from_cam_to_robot(self, pose2cam):
        """transform pose from cam-coordinate to robot-coordinate"""
        cam2world = self.get_prim_world_pose(prim_path=self.base_camera, camera=True)
        pose2world = cam2world @ pose2cam
        return pose2world

    def pose_from_world_to_cam(self, pose2world):
        """transform pose from world-coordinate to cam-coordinate"""
        cam2world = self.get_prim_world_pose(prim_path=self.base_camera, camera=True)
        pose2cam = np.linalg.inv(cam2world) @ pose2world
        return pose2cam

    def decode_gripper_pose(self, gripper_pose):
        """Decode gripper-pose at cam-coordinate to end-pose at robot-coordinate"""
        gripper_pose = self.pose_from_cam_to_robot(gripper_pose)
        angle = 0  # np.pi / 2
        rot_z = np.array(
            [
                [np.cos(angle), -np.sin(angle), 0],
                [np.sin(angle), np.cos(angle), 0],
                [0, 0, 1],
            ]
        )

        flange2gripper = np.eye(4)
        flange2gripper[:3, :3] = rot_z
        flange2gripper[2, 3] = -0.01
        return gripper_pose @ flange2gripper

    def get_ee_pose(self, id="right", **kwargs):
        state = self.client.GetEEPose(is_right=id == "right")
        xyz = np.array(
            [
                state.ee_pose.position.x,
                state.ee_pose.position.y,
                state.ee_pose.position.z,
            ]
        )
        quat = np.array(
            [
                state.ee_pose.rpy.rw,
                state.ee_pose.rpy.rx,
                state.ee_pose.rpy.ry,
                state.ee_pose.rpy.rz,
            ]
        )
        pose = np.eye(4)
        pose[:3, 3] = xyz
        pose[:3, :3] = quat_to_rot_matrix(quat)
        return pose

    def init_transform(self):
        init_rotation = quat_to_rot_matrix(self.init_rotation)
        translation_matrix = np.zeros((4, 4))
        translation_matrix[:3, :3] = init_rotation
        translation_matrix[:3, 3] = self.init_position
        translation_matrix[3, 3] = 1
        self.ego_transform = translation_matrix

    def get_init_pose(self):
        return self.init_position, self.init_rotation

    def update_transform(self):
        base_pos, base_rot = self.get_base_pose()
        base_rot_matrix = quat_to_rot_matrix(base_rot)
        translation_matrix = np.zeros((4, 4))
        translation_matrix[:3, :3] = base_rot_matrix
        translation_matrix[:3, 3] = base_pos
        translation_matrix[3, 3] = 1
        self.ego_transform = translation_matrix

    def transform_from_world_to_base(self, pos, rot):
        world_pose = np.zeros((4, 4))
        world_pose[:3, :3] = get_rotation_matrix_from_euler(rot)
        world_pose[:3, 3] = pos
        world_pose[3, 3] = 1
        local_pose = np.linalg.inv(self.ego_transform) @ world_pose
        return local_pose[:3, 3], get_quaternion_from_rotation_matrix(
            local_pose[:3, :3]
        )

    def transform_from_base_to_world(self, pos, rot):
        base_pose = np.zeros((4, 4))
        base_pose[:3, :3] = get_rotation_matrix_from_euler(rot)
        base_pose[:3, 3] = pos
        base_pose[3, 3] = 1
        world_pose = self.ego_transform @ base_pose
        return world_pose[:3, 3], get_quaternion_from_rotation_matrix(
            world_pose[:3, :3]
        )

    def initialize_solver(self, joint_info):
        head_init_position = joint_info["ori_pos"][:2]
        waist_init_position = joint_info["ori_pos"][2:4]
        l_arm = joint_info["l_arm"]
        r_arm = joint_info["r_arm"]
        self.ik_solver.initialize_states(
            left_arm_init=np.array(l_arm, dtype=np.float32),
            right_arm_init=np.array(r_arm, dtype=np.float32),
            head_init=np.array(head_init_position, dtype=np.float32),
        )
        q_full = np.zeros(18)
        q_full[0] = waist_init_position[1]
        q_full[1] = waist_init_position[0]
        self.center_T_base = self.ik_solver.compute_fk(
            q=q_full, start_link="base_link", end_link="arm_base_link"
        )
        self.base_T_center = np.linalg.inv(self.center_T_base)

    def get_joint_from_deltapos(self, xyz, rpy, id, isOn):
        idx = 0 if id == "left" else 1
        part = (
            ik_solver.RobotPart.LEFT_ARM
            if id == "left"
            else ik_solver.RobotPart.RIGHT_ARM
        )
        if isOn and self.last_eef_pos[idx]:
            pos, rot = deepcopy(self.last_eef_pos[idx])
        else:
            self.last_eef_pos[idx] = self.ik_solver.get_current_target(part)
            return np.array([])

        pos += xyz
        rot_new = Rotation.from_euler(
            "xyz", matrix_to_euler_angles(rot) + rpy, degrees=False
        ).as_matrix()

        self.ik_solver.update_target_mat(
            part=part,
            target_pos=pos,
            target_rot=rot_new,
        )
        if id == "left":
            ret = self.ik_solver.solve_left_arm()
        else:
            ret = self.ik_solver.solve_right_arm()
        return np.array(ret.tolist())

    def check_ik(self, pose, id="right", **kwargs):
        xyz, quat = pose[:3, 3], get_quaternion_from_rotation_matrix(pose[:3, :3])
        state = self.client.GetIKStatus(
            target_position=xyz, target_rotation=quat, is_right=id == "right"
        )
        return state.isSuccess

    def solve_ik(self, pose, arm="right", type="Simple", **kwargs):
        single_mode = len(pose.shape) == 2
        if single_mode:
            pose = pose[np.newaxis, ...]

        xyz, quat = pose[:, :3, 3], get_quaternion_wxyz_from_rotation_matrix(
            pose[:, :3, :3]
        )
        target_poses = []
        for i in range(len(pose)):
            pose = {"position": xyz[i], "rotation": quat[i]}
            target_poses.append(pose)

        ObsAvoid = type == "ObsAvoid" or type == "AvoidObs"
        result = self.client.GetIKStatus(
            target_poses=target_poses, is_right=arm == "right", ObsAvoid=ObsAvoid
        )  # True: isaac,  False: curobo
        ik_result = []
        jacobian_score = []
        joint_positions = []
        joint_names = []
        for state in result:
            ik_result.append(state["status"])
            jacobian_score.append(state["Jacobian"])
            joint_positions.append(state["joint_positions"])
            joint_names.append(state["joint_names"])
        # result = np.array(result)
        if single_mode:
            ik_result = ik_result[0]
            jacobian_score = jacobian_score[0]
            joint_positions = joint_positions[0]
            joint_names = joint_names[0]
        info = {
            "jacobian_score": np.array(jacobian_score),
            "joint_positions": np.array(joint_positions),
            "joint_names": np.array(joint_names),
        }
        return np.array(ik_result), info

    def get_joint_pose(self):
        joint_pos = self.client.get_joint_positions()
        return joint_pos

    def set_joint_pose(self, target_joint_position, is_trajectory=False):
        self.client.set_joint_positions(
            target_joint_position=target_joint_position, is_trajectory=is_trajectory
        )

    def set_base_pose(self, target_pos, target_rot):
        self.client.SetObjectPose(
            [{"prim_path": "robot", "position": target_pos, "rotation": target_rot}], []
        )

    def get_base_pose(self):
        rsp = self.client.get_object_pose("robot")
        position = (
            rsp.object_pose.position.x,
            rsp.object_pose.position.y,
            rsp.object_pose.position.z,
        )
        rotation = (
            rsp.object_pose.rpy.rw,
            rsp.object_pose.rpy.rx,
            rsp.object_pose.rpy.ry,
            rsp.object_pose.rpy.rz,
        )
        return position, rotation
