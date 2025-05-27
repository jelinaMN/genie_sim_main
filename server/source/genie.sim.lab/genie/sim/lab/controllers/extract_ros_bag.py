# Copyright (c) 2023-2025, AgiBot Inc. All Rights Reserved.
# Author: Genie Sim Team
# License: Mozilla Public License Version 2.0

import os, sys
import json, h5py
import numpy as np
import subprocess
import cv2

from pathlib import Path
from rosbags.highlevel import AnyReader
from rosbags.image import message_to_cvimage

from base_utils.logger import Logger

logger = Logger()  # Create singleton instance


count = 0


class Ros_Extrater:
    def __init__(
        self,
        bag_file,
        output_dir,
        robot_init_position,
        robot_init_rotation,
        camera_info={},
        robot_name="G1",
        scene_name="test",
        scene_usd="",
        object_names={},
        frame_status=[],
        fps=30,
    ):
        self.bag_file = bag_file
        self.output_dir = output_dir
        self.robot_init_position = robot_init_position
        self.robot_init_rotation = robot_init_rotation
        self.camera_info = camera_info
        self.robot_name = robot_name
        self.scene_name = scene_name
        self.scene_usd = scene_usd
        self.object_names = []
        for name in object_names:
            if name not in self.object_names:
                self.object_names.append(name)
        self.frame_status = frame_status
        self.fps = fps

    def post_process_file_name(self, file_name, extra_name="", remove_name=False):
        if remove_name:
            index = file_name.rfind("_")
            file_name = file_name[:index] + extra_name
        else:
            file_name = file_name + extra_name
        return file_name

    def extract(self):
        with AnyReader([Path(self.bag_file)]) as reader:
            image_topics = {}
            joint_topics = {}
            tf_topics = {}
            rgb_topics = {}
            message_step = np.inf
            physics_message_step = np.inf
            self.imag_file_name = []
            for connection in reader.connections:
                if connection.msgtype == "sensor_msgs/msg/Image":
                    image_topics[connection.topic] = []
                    if connection.msgcount < message_step:
                        message_step = connection.msgcount
                elif connection.msgtype == "sensor_msgs/msg/JointState":
                    joint_topics[connection.topic] = []
                    if connection.msgcount < physics_message_step:
                        physics_message_step = connection.msgcount
                elif connection.topic == "/obj_tf":
                    tf_topics[connection.topic] = []
                    if connection.msgcount < physics_message_step:
                        physics_message_step = connection.msgcount
            # init_image
            init_time = {}
            label_dict = None
            for connection, timestamp, msg in reader.messages():
                if connection.msgtype == "sensor_msgs/msg/Image":
                    image_topics[connection.topic].append(msg)
                elif connection.msgtype == "sensor_msgs/msg/JointState":
                    joint_topics[connection.topic].append(msg)
                elif connection.topic == "/obj_tf":
                    tf_topics[connection.topic].append(msg)
                elif connection.msgtype == "std_msgs/msg/String":
                    lable_msg = reader.deserialize(msg, "std_msgs/msg/String").data
                    # label_dict = json.loads(lable_msg)
                elif connection.msgtype == "sensor_msgs/msg/CompressedImage":
                    rgb_msg = reader.deserialize(msg, "sensor_msgs/msg/CompressedImage")
                    current_time = (float)(rgb_msg.header.stamp.sec) + (float)(
                        rgb_msg.header.stamp.nanosec
                    ) * np.power(10.0, -9)
                    file_name = rgb_msg.header.frame_id
                    file_name = self.post_process_file_name(file_name)
                    if file_name not in rgb_topics:
                        rgb_topics[file_name] = [msg]
                        init_time[file_name] = current_time
                        self.imag_file_name.append(file_name)
                    else:
                        if current_time != init_time[file_name]:
                            rgb_topics[file_name].append(msg)
                            init_time[file_name] = current_time

            # init_start_time
            physics_start_time = 0
            for key in image_topics:
                if len(image_topics[key]) < 1:
                    return
                msg = reader.deserialize(image_topics[key][0], "sensor_msgs/msg/Image")
                image_time_stamp = (float)(msg.header.stamp.sec) + (float)(
                    msg.header.stamp.nanosec
                ) * np.power(10.0, -9)
                if image_time_stamp > physics_start_time:
                    physics_start_time = image_time_stamp
            for key in rgb_topics:
                if len(rgb_topics[key]) < 1:
                    return
                msg = reader.deserialize(
                    rgb_topics[key][0], "sensor_msgs/msg/CompressedImage"
                )
                image_time_stamp = (float)(msg.header.stamp.sec) + (float)(
                    msg.header.stamp.nanosec
                ) * np.power(10.0, -9)
                if image_time_stamp > physics_start_time:
                    physics_start_time = image_time_stamp
            for key in joint_topics:
                if len(joint_topics[key]) < 1:
                    return
                msg = reader.deserialize(
                    joint_topics[key][0], "sensor_msgs/msg/JointState"
                )
                joint_time_stamp = (float)(msg.header.stamp.sec) + (float)(
                    msg.header.stamp.nanosec
                ) * np.power(10.0, -9)
                if joint_time_stamp > physics_start_time:
                    physics_start_time = joint_time_stamp
            for key in tf_topics:
                if len(tf_topics[key]) < 1:
                    return
                msg = reader.deserialize(tf_topics[key][0], "tf2_msgs/msg/TFMessage")
                tf_time_stamp = (float)(msg.transforms[0].header.stamp.sec) + (float)(
                    msg.transforms[0].header.stamp.nanosec
                ) * np.power(10.0, -9)
                if tf_time_stamp > physics_start_time:
                    physics_start_time = tf_time_stamp

            # alignment
            for key in image_topics:
                msg = reader.deserialize(image_topics[key][0], "sensor_msgs/msg/Image")
                image_time_stamp = (float)(msg.header.stamp.sec) + (float)(
                    msg.header.stamp.nanosec
                ) * np.power(10.0, -9)
                while image_time_stamp < physics_start_time:
                    del image_topics[key][0]
                    if len(image_topics[key]) < 1:
                        return
                    msg = reader.deserialize(
                        image_topics[key][0], "sensor_msgs/msg/Image"
                    )
                    image_time_stamp = (float)(msg.header.stamp.sec) + (float)(
                        msg.header.stamp.nanosec
                    ) * np.power(10.0, -9)
                if len(image_topics[key]) < message_step:
                    message_step = len(image_topics[key])
            for key in rgb_topics:
                msg = reader.deserialize(
                    rgb_topics[key][0], "sensor_msgs/msg/CompressedImage"
                )
                image_time_stamp = (float)(msg.header.stamp.sec) + (float)(
                    msg.header.stamp.nanosec
                ) * np.power(10.0, -9)
                while image_time_stamp < physics_start_time:
                    del rgb_topics[key][0]
                    if len(rgb_topics[key]) < 1:
                        return
                    msg = reader.deserialize(
                        rgb_topics[key][0], "sensor_msgs/msg/CompressedImage"
                    )
                    image_time_stamp = (float)(msg.header.stamp.sec) + (float)(
                        msg.header.stamp.nanosec
                    ) * np.power(10.0, -9)
                if len(rgb_topics[key]) < message_step:
                    message_step = len(rgb_topics[key])
            for key in joint_topics:
                while joint_time_stamp < physics_start_time:
                    del joint_topics[key][0]
                    msg = reader.deserialize(
                        joint_topics[key][0], "sensor_msgs/msg/JointState"
                    )
                    joint_time_stamp = (float)(msg.header.stamp.sec) + (float)(
                        msg.header.stamp.nanosec
                    ) * np.power(10.0, -9)
                if len(joint_topics[key]) < physics_message_step:
                    physics_message_step = len(joint_topics[key])
            for key in tf_topics:
                while tf_time_stamp < physics_start_time:
                    del tf_topics[key][0]
                    msg = reader.deserialize(
                        tf_topics[key][0], "tf2_msgs/msg/TFMessage"
                    )
                    tf_time_stamp = (float)(msg.transforms[0].header.stamp.sec) + (
                        float
                    )(msg.transforms[0].header.stamp.nanosec) * np.power(10.0, -9)
                if len(tf_topics[key]) < physics_message_step:
                    physics_message_step = len(tf_topics[key])

            render_time_step = []
            if message_step > 0 and message_step < 1000000:
                for idx in range(message_step):
                    depth_dir = self.output_dir + "/camera/{}".format(idx)
                    rgb_dir = self.output_dir + "/camera/{}".format(idx)
                    os.makedirs(depth_dir, exist_ok=True)
                    os.makedirs(rgb_dir, exist_ok=True)
                    stamp = {}
                    for key in image_topics:
                        msg = reader.deserialize(
                            image_topics[key][idx], "sensor_msgs/msg/Image"
                        )
                        file_name = key.split("/")[-1]
                        stamp[file_name] = (float)(msg.header.stamp.sec) + (float)(
                            msg.header.stamp.nanosec
                        ) * np.power(10.0, -9)
                        if "depth" in file_name:
                            img = message_to_cvimage(msg, "32FC1") * 1000
                            file_name = self.post_process_file_name(
                                file_name, remove_name=True
                            )
                            cv2.imwrite(
                                depth_dir + "/{}.png".format(file_name),
                                img.astype(np.uint16),
                            )
                        elif "semantic" in file_name:
                            img = message_to_cvimage(msg, "32FC1")
                            img = np.where((img == 0) | (img == 1), 0, img - 1)
                            file_name = self.post_process_file_name(
                                file_name, "_semantic"
                            )
                            cv2.imwrite(depth_dir + "/{}.png".format(file_name), img)
                    for key in rgb_topics:
                        msg = reader.deserialize(
                            rgb_topics[key][idx], "sensor_msgs/msg/CompressedImage"
                        )
                        file_name = key.split("/")[-1]
                        stamp[file_name] = (float)(msg.header.stamp.sec) + (float)(
                            msg.header.stamp.nanosec
                        ) * np.power(10.0, -9)
                        img = message_to_cvimage(
                            msg, "bgr8"
                        )  # change encoding type if needed
                        file_name = self.post_process_file_name(file_name)
                        cv2.imwrite(rgb_dir + "/{}.jpg".format(file_name), img)
                    min_value = min(stamp.values())
                    for key in stamp:
                        stamp[key] = min_value
                    with open(
                        depth_dir + "/time_stamp.json", "w", encoding="utf-8"
                    ) as f:
                        json.dump(stamp, f, indent=4)
                    render_time_step.append(str(list(stamp.values())[0]))

            def get_pose(xyz: np.ndarray, quat_wxyz: np.ndarray) -> np.ndarray:
                def get_rotation_matrix_from_quaternion(quat: np.ndarray) -> np.ndarray:
                    w, x, y, z = quat
                    rot = np.array(
                        [
                            [
                                2 * (w**2 + x**2) - 1,
                                2 * (x * y - w * z),
                                2 * (x * z + w * y),
                            ],
                            [
                                2 * (x * y + w * z),
                                2 * (w**2 + y**2) - 1,
                                2 * (y * z - w * x),
                            ],
                            [
                                2 * (x * z - w * y),
                                2 * (y * z + w * x),
                                2 * (w**2 + z**2) - 1,
                            ],
                        ]
                    )
                    return rot

                pose = np.eye(4)
                pose[:3, :3] = get_rotation_matrix_from_quaternion(quat_wxyz)
                pose[:3, 3] = xyz
                return pose

            result = {
                "scene": {
                    "name": self.scene_name,
                    "metadata": None,
                    "scene_usd": self.scene_usd,
                },
                "objects": [],
                "cameras": self.camera_info,
                "robot": {"name": self.robot_name, "metadata": None},
                "frames": [],
                "fps": self.fps,
            }
            for name in self.object_names:
                result["objects"].append(
                    {"name": name.split("/")[-1], "metadata": None}
                )
            state_info = {"timestamp": []}
            episode_state = {
                "joint": {
                    "effort": [],
                    "position": [],
                    "velocity": [],
                    "current_value": [],
                },
                "end": {
                    "velocity": [],
                    "angular": [],
                    "position": [],
                    "orientation": [],
                    "wrench": [],
                },
                "effector": {"force": [], "position": [], "index": []},
            }
            attr_names = {
                "joint": [],
                "end": ["left", "right"],
                "effector": [],
                "robot": [self.robot_name],
            }
            robot_info = {
                "position": [],
                "orientation": [],
                "position_drift": [],
                "orientation_drift": [],
                "velocity": [],
            }
            for idx in range(physics_message_step):
                single_frame_state = {
                    "objects": {},
                    "articulated_object": {},
                    "cameras": {},
                    "ee": {},
                    "robot": {},
                }
                art_key = ""
                for key in joint_topics:
                    if key == "/joint_states":
                        msg = reader.deserialize(
                            joint_topics[key][idx], "sensor_msgs/msg/JointState"
                        )
                        joint_timestamp = (float)(msg.header.stamp.sec) + (float)(
                            msg.header.stamp.nanosec
                        ) * np.power(10.0, -9)
                        single_joint_info = {
                            "joint_name": msg.name,
                            "joint_position": msg.position.tolist(),
                            "joint_velocity": msg.velocity.tolist(),
                            "joint_effort": msg.effort.tolist(),
                        }
                    else:
                        msg = reader.deserialize(
                            joint_topics[key][idx], "sensor_msgs/msg/JointState"
                        )
                        joint_timestamp = (float)(msg.header.stamp.sec) + (float)(
                            msg.header.stamp.nanosec
                        ) * np.power(10.0, -9)
                        single_frame_state["articulated_object"][key.split("/")[-1]] = {
                            "joints": {
                                "joint_name": msg.name,
                                "joint_position": msg.position.tolist(),
                                "joint_velocity": msg.velocity.tolist(),
                                "joint_effort": msg.effort.tolist(),
                            }
                        }
                        art_key = key.split("/")[-1]
                single_frame_state["time_stamp"] = joint_timestamp
                frame_idx = -1
                if len(self.frame_status) > 0:
                    if joint_timestamp < self.frame_status[0]["time_stamp"]:
                        value = self.frame_status[0]["frame_state"]
                    else:
                        while (
                            joint_timestamp < self.frame_status[frame_idx]["time_stamp"]
                        ):
                            frame_idx -= 1
                            value = self.frame_status[frame_idx]["frame_state"]
                    single_frame_state["frame_state"] = value
                single_frame_state["robot"]["joints"] = single_joint_info
                single_ee_info_r = {
                    "time_stamp": 0,
                    "position": [0, 0, 0],
                    "rotation": [1, 0, 0, 0],
                }
                single_ee_info_l = {
                    "time_stamp": 0,
                    "position": [0, 0, 0],
                    "rotation": [1, 0, 0, 0],
                }
                for key in tf_topics:
                    msg = reader.deserialize(
                        tf_topics[key][idx], "tf2_msgs/msg/TFMessage"
                    )
                    single_object_info = {}
                    for transform in msg.transforms:
                        position = np.array(
                            [
                                transform.transform.translation.x,
                                transform.transform.translation.y,
                                transform.transform.translation.z,
                            ]
                        )
                        rotation = np.array(
                            [
                                transform.transform.rotation.w,
                                transform.transform.rotation.x,
                                transform.transform.rotation.y,
                                transform.transform.rotation.z,
                            ]
                        )
                        if "right_gripper_center" in transform.child_frame_id:
                            single_ee_info_r = {
                                "time_stamp": (float)(transform.header.stamp.sec)
                                + (float)(transform.header.stamp.nanosec)
                                * np.power(10.0, -9),
                                "position": position,
                                "rotation": rotation,
                            }
                            single_frame_state["ee"]["right"] = {
                                "pose": (get_pose(*(position, rotation)).tolist())
                            }
                        elif "gripper_center" in transform.child_frame_id:
                            single_ee_info_l = {
                                "time_stamp": (float)(transform.header.stamp.sec)
                                + (float)(transform.header.stamp.nanosec)
                                * np.power(10.0, -9),
                                "position": position,
                                "rotation": rotation,
                            }
                            single_frame_state["ee"]["left"] = {
                                "pose": (get_pose(*(position, rotation)).tolist())
                            }
                        elif "base_link" in transform.child_frame_id:
                            single_frame_state["robot"][
                                "pose"
                            ] = get_pose(*(position, rotation)).tolist()
                        elif (
                            "Camera" in transform.child_frame_id
                            or "Fisheye" in transform.child_frame_id
                        ):
                            rotation_x_180 = np.array(
                                [
                                    [1.0, 0.0, 0.0, 0],
                                    [0.0, -1.0, 0.0, 0],
                                    [0.0, 0.0, -1.0, 0],
                                    [0, 0, 0, 1],
                                ]
                            )
                            camera_key = transform.child_frame_id

                            camera_key = self.post_process_file_name(camera_key)
                            single_frame_state["cameras"][camera_key] = {
                                "pose": (
                                    get_pose(*(position, rotation)) @ rotation_x_180
                                ).tolist()
                            }
                        elif "selfModeling_table_001_Base" in transform.child_frame_id:
                            single_frame_state["articulated_object"][art_key][
                                "pose"
                            ] = get_pose(*(position, rotation)).tolist()
                        else:
                            if (
                                "selfModeling_table" not in transform.child_frame_id
                                and "Handle" not in transform.child_frame_id
                            ):
                                single_frame_state["objects"][
                                    transform.child_frame_id
                                ] = {"pose": get_pose(*(position, rotation)).tolist()}
                if str(joint_timestamp) in render_time_step:
                    result["frames"].append(single_frame_state)
                    # align hdf5
                    state_info["timestamp"].append(joint_timestamp)
                    episode_state["joint"]["position"].append(
                        single_joint_info["joint_position"]
                    )
                    episode_state["joint"]["velocity"].append(
                        single_joint_info["joint_velocity"]
                    )
                    episode_state["joint"]["effort"].append(
                        single_joint_info["joint_effort"]
                    )
                    if not attr_names["joint"]:
                        attr_names["joint"] = single_joint_info["joint_name"]
                    episode_state["end"]["position"].append(
                        [single_ee_info_l["position"], single_ee_info_r["position"]]
                    )
                    episode_state["end"]["orientation"].append(
                        [single_ee_info_l["rotation"], single_ee_info_r["rotation"]]
                    )
                    # better joint_position to width
                    if len(single_joint_info["joint_position"]) > 18:
                        episode_state["effector"]["position"].append(
                            [
                                (single_joint_info["joint_position"][18] + 1) * 120,
                                (single_joint_info["joint_position"][22] + 1) * 120,
                            ]
                        )
                    episode_state["effector"]["index"].append(idx)
                    episode_state["effector"]["force"].append(0)
                    if not attr_names["effector"]:
                        attr_names["effector"] = ["left", "right"]
                    robot_info["position"].append(self.robot_init_position)
                    robot_info["orientation"].append(self.robot_init_rotation)
                    robot_info["orientation_drift"].append([1, 0, 0, 0])
                    robot_info["position_drift"].append([0, 0, 0])
                    robot_info["velocity"].append(0.0)

            state_info["state"] = {
                "joint": episode_state["joint"],
                "end": episode_state["end"],
                "effector": episode_state["effector"],
            }
            state_info["action"] = {
                "joint": episode_state["joint"],
                "end": episode_state["end"],
                "effector": episode_state["effector"],
            }
            state_info["state"]["robot"] = {
                "position": robot_info["position"],
                "orientation": robot_info["orientation"],
                "position_drift": robot_info["position_drift"],
                "orientation_drift": robot_info["orientation_drift"],
            }
            state_info["action"]["robot"] = {
                "velocity": robot_info["velocity"],
                "orientation": robot_info["orientation"],
            }

            with h5py.File(self.output_dir + "/aligned_joints.h5", "w") as hdf:
                hdf.create_dataset(
                    "timestamp", data=np.array(state_info["timestamp"], dtype="float32")
                )
                state_group = hdf.create_group("state")
                for state_key, state_value in state_info["state"].items():
                    group = state_group.create_group(state_key)
                    if state_key == "joint":
                        group.attrs["name"] = attr_names["joint"]
                    elif state_key == "end":
                        group.attrs["name"] = attr_names["end"]
                    elif state_key == "effector":
                        group.attrs["name"] = attr_names["effector"]
                        group.attrs["category"] = ["continuous"]
                    elif state_key == "robot":
                        group.attrs["name"] = attr_names["robot"]
                    for inner_key, value in state_value.items():
                        if isinstance(value, (int, float, bool)):
                            dataset = group.create_dataset(inner_key, data=value)
                        elif isinstance(value, str):
                            dataset = group.create_dataset(
                                inner_key, data=np.string_(value)
                            )
                        elif isinstance(value, list):
                            dataset = group.create_dataset(
                                inner_key, data=np.array(value, dtype="float32")
                            )

                state_group_1 = hdf.create_group("action")
                for state_key, state_value in state_info["action"].items():
                    group = state_group_1.create_group(state_key)
                    if state_key == "joint":
                        group.attrs["name"] = attr_names["joint"]
                    elif state_key == "end":
                        group.attrs["name"] = attr_names["end"]
                    elif state_key == "effector":
                        group.attrs["name"] = attr_names["effector"]
                        group.attrs["category"] = ["continuous"]
                    elif state_key == "robot":
                        group.attrs["name"] = attr_names["robot"]
                    for inner_key, value in state_value.items():
                        if isinstance(value, (int, float, bool)):
                            dataset = group.create_dataset(inner_key, data=value)
                        elif isinstance(value, str):
                            dataset = group.create_dataset(
                                inner_key, data=np.string_(value)
                            )
                        elif isinstance(value, list):
                            dataset = group.create_dataset(
                                inner_key, data=np.array(value, dtype="float32")
                            )
            if label_dict:
                lable_result = []
                for key, value in label_dict.items():
                    if key != "1" and key != "time_stamp":
                        idx = (int)(key) - 1
                        if idx < 0:
                            idx = 0
                        value["id"] = idx
                        lable_result.append(value)
                result["semantic_lables"] = lable_result
            with open(self.output_dir + "/state.json", "w", encoding="utf-8") as f:
                json.dump(result, f, indent=4)
            os.makedirs(self.output_dir + "/parameters/camera", exist_ok=True)
            with open(
                self.output_dir + "/parameters/camera/state.json", "w", encoding="utf-8"
            ) as f:
                json.dump(result, f, indent=4)

            try:
                logger.info(self.output_dir)
                file_name = self.output_dir.split("/")[-1] + "_0"
                for image_file in self.imag_file_name:
                    subprocess.run(
                        [
                            "ffmpeg",
                            "-framerate",
                            "30",
                            "-i",
                            f"{self.output_dir}/camera/%d/{image_file}.jpg",
                            "-c:v",
                            "libx265",
                            "-b:v",
                            "3000k",
                            "-preset",
                            "slow",
                            "-crf",
                            "18",
                            f"{self.output_dir}/{image_file}.mp4",
                        ]
                    )
                logger.info(f"Video file saved to {self.output_dir}")
                subprocess.run(["rm", "-Rf", f"{self.output_dir}/{file_name}.db3"])
                logger.info(f"Successfully transfer h265")
            except subprocess.CalledProcessError as e:
                logger.error(f"Error removing file: {e}")
                sys.exit(1)
