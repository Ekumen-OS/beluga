# Copyright 2024 Ekumen, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from dataclasses import dataclass
from pathlib import Path
import typing
import rclpy.time
import rosbag2_py

import numpy as np
from sensor_msgs.msg import PointCloud2
from rclpy.serialization import deserialize_message
from scipy.spatial.transform import Rotation

import sensor_msgs_py.point_cloud2 as pc2
from rosidl_runtime_py.utilities import get_message
import rclpy
import open3d as o3d
from tf2_msgs.msg import TFMessage
import sensor_msgs_py.numpy_compat as nc
import logging
import time

import argparse

import tf2_ros


class ThrottledHandler(logging.Handler):
    """Handler that throttles logging to at most every N seconds."""

    def __init__(self, rate_limit_seconds: float):
        """Constucts the handler."""
        super().__init__()
        self.rate_limit_seconds: float = rate_limit_seconds
        self.last_logged_time: float = 0.0

    def emit(self, record: logging.LogRecord):
        """Optionally emit the record."""
        current_time = time.time()
        if current_time - self.last_logged_time >= self.rate_limit_seconds:
            self.last_logged_time = current_time
            # Here you can format the record and output it as needed
            self.handle(record)


def lookup_transform(
    tf_buffer: tf2_ros.Buffer,
    target_frame: str,
    source_frame: str,
    time: rclpy.time.Time,
) -> typing.Optional[tf2_ros.TransformStamped]:
    """Look up a transform between target_frame and source_frame at 'time' in 'tf_buffer'."""
    try:
        transform_stamped = tf_buffer.lookup_transform(target_frame, source_frame, time)
        return transform_stamped.transform
    except tf2_ros.LookupException as e:
        print(f"error : {e=}")
        return None


def get_rosbag_options(
    path: Path, storage_id: str = "sqlite3", serialization_format: str = "cdr"
) -> typing.Tuple[rosbag2_py.StorageOptions, rosbag2_py.ConverterOptions]:
    """Return storage and converter options used for reading the bag."""
    storage_options = rosbag2_py.StorageOptions(
        uri=str(path.absolute()), storage_id=storage_id
    )

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format,
    )

    return storage_options, converter_options


def get_tf_buffer(reader: rosbag2_py.SequentialReader) -> tf2_ros.Buffer:
    """Return the transform buffer from a rosbag reader."""
    storage_filter = rosbag2_py.StorageFilter(topics=["/tf", "/tf_static"])
    reader.set_filter(storage_filter)
    topic_types = reader.get_all_topics_and_types()
    type_map = {
        topic_types[i].name: topic_types[i].type for i in range(len(topic_types))
    }
    tf_buffer = tf2_ros.Buffer()
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg: TFMessage = deserialize_message(data, msg_type)
        for tf in msg.transforms:
            if topic == "/tf_static":
                tf_buffer.set_transform_static(transform=tf, authority="test")
            else:
                tf_buffer.set_transform(transform=tf, authority="test")
    return tf_buffer


def get_matrix(msg: tf2_ros.TransformStamped) -> np.ndarray:
    """Return the 4X4 transformation matrix from a transform ROS message."""
    mat = np.identity(4, dtype=float)
    rotation = Rotation.from_quat(
        [
            msg.transform.rotation.x,
            msg.transform.rotation.y,
            msg.transform.rotation.z,
            msg.transform.rotation.w,
        ]
    )
    mat[0:3, 0:3] = rotation.as_matrix()
    mat[0, 3] = msg.transform.translation.x
    mat[1, 3] = msg.transform.translation.y
    mat[2, 3] = msg.transform.translation.z
    return mat


@dataclass
class Arguments:
    bag: Path
    scan_topic: str
    frame: str
    output: Path
    voxel_size: float


def parse_args() -> Arguments:
    arg_parser = argparse.ArgumentParser(usage=__doc__)
    arg_parser.add_argument(
        "--bag",
        "-b",
        type=Path,
        help="Path to the ROS bag to use for scan aggregation.",
        required=True,
    )
    arg_parser.add_argument(
        "--output",
        "-o",
        type=Path,
        help="Path to the file, or directory where outputs will be dumped to. \
          Will be created if it doesn't exist.",
        required=True,
    )
    arg_parser.add_argument(
        "--scan_topic",
        "-s",
        type=str,
        help="Scan topic containing PointCloud2 messages to be aggregated.",
        required=True,
    )
    arg_parser.add_argument(
        "--voxel_size",
        "-v",
        type=float,
        default=0.025,
        help="3D map will be decimated using a voxel filter of this size.",
    )
    arg_parser.add_argument(
        "--frame",
        "-f",
        type=str,
        required=True,
        help="Frame in which the clouds will be aggregated, for instance, 'odom'",
    )
    return Arguments(**vars(arg_parser.parse_args()))


def main():
    args = parse_args()
    # Example usage
    logging.basicConfig()
    logger = logging.getLogger('throttled_logger')
    logger.setLevel(logging.INFO)

    throttled_handler = ThrottledHandler(
        rate_limit_seconds=5
    )  # Throttle to 1 message every 5 seconds
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    throttled_handler.setFormatter(formatter)
    logger.addHandler(throttled_handler)

    assert args.bag.exists(), f"Bag path doesn't exist: {args.bag}"
    assert args.voxel_size > 0, f"Invalid voxel size {args.voxel_size}"

    output_file = (
        args.output / "aggregated_scans.ply" if args.output.is_dir() else args.output
    )
    output_file.parent.mkdir(parents=True, exist_ok=True)

    storage_options, converter_options = get_rosbag_options(args.bag)

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    tf_tree = get_tf_buffer(reader=reader)
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    storage_filter = rosbag2_py.StorageFilter(topics=[args.scan_topic])
    reader.set_filter(storage_filter)
    topic_types = reader.get_all_topics_and_types()
    type_map = {
        topic_types[i].name: topic_types[i].type for i in range(len(topic_types))
    }

    map_points = o3d.geometry.PointCloud()

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg: PointCloud2 = deserialize_message(data, msg_type)
        time = rclpy.time.Time.from_msg(msg.header.stamp)
        try:
            tf = tf_tree.lookup_transform(args.frame, msg.header.frame_id, time)
            points = nc.structured_to_unstructured(
                pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            ).T
            matrix = get_matrix(tf)
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points.T)
            pcd.remove_non_finite_points()
            pcd = pcd.voxel_down_sample(args.voxel_size)
            pcd.transform(matrix)
            map_points += pcd
        except Exception as e:
            print(e)
            pass
    o3d.io.write_point_cloud(
        str(output_file.absolute()), map_points.voxel_down_sample(args.voxel_size)
    )
