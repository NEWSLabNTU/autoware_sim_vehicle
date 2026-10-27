# Copyright 2020 Tier IV, Inc. All rights reserved.
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

"""
Simulated LiDAR Pointcloud Preprocessor Launch

For a simulated car with a single LiDAR, the concatenation filter (which needs 2+ LiDARs) is
replaced by a crop box that relays the LiDAR to the concatenated output topic downstream
Autoware reads, without the points inside the car itself: the box is the car's vehicle_info
(tesla_model3_description) plus SELF_MARGIN, in base_link, as a real sensor kit's self crop. A car
whose roof LiDAR sees its own hood would otherwise see an obstacle inside its footprint, and
the obstacle stop trims its trajectory to nothing (swb roadmap 057: drift_mustang's long hood
at 3.4 m, z 0.8). A car whose LiDAR sees none of itself loses nothing.
"""

import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
import os
import yaml
from ament_index_python.packages import get_package_share_directory

SELF_MARGIN = 0.2  # m around the car's box


def self_box():
    """The car's box in base_link (rear axle centre on the ground), from its vehicle_info."""
    path = os.path.join(get_package_share_directory("tesla_model3_description"), "config",
                        "vehicle_info.param.yaml")
    v = yaml.safe_load(open(path))["/**"]["ros__parameters"]
    half_width = v["wheel_tread"] / 2 + max(v["left_overhang"], v["right_overhang"])
    m = SELF_MARGIN
    return {"min_x": -v["rear_overhang"] - m, "max_x": v["wheel_base"] + v["front_overhang"] + m,
            "min_y": -half_width - m, "max_y": half_width + m,
            "min_z": -m, "max_z": v["vehicle_height"] + m}


def launch_setup(context, *args, **kwargs):
    # The self crop box as the relay for the single LiDAR: /sensing/lidar/top/pointcloud_before_sync
    # to concatenated/pointcloud, in base_link, without the points inside the car
    passthrough_component = ComposableNode(
        package="autoware_pointcloud_preprocessor",
        plugin="autoware::pointcloud_preprocessor::CropBoxFilterComponent",
        name="crop_box_filter_self",
        remappings=[
            ("input", "/sensing/lidar/top/pointcloud_before_sync"),
            ("output", "concatenated/pointcloud"),
        ],
        parameters=[
            {
                "input_frame": LaunchConfiguration("base_frame"),
                "output_frame": LaunchConfiguration("base_frame"),
                "negative": True,  # keep what is outside the box
                "processing_time_threshold_sec": 0.01,
                **self_box(),
            }
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    # Load passthrough filter
    passthrough_loader = LoadComposableNodes(
        composable_node_descriptions=[passthrough_component],
        target_container=LaunchConfiguration("pointcloud_container_name"),
        condition=IfCondition(LaunchConfiguration("use_concat_filter")),
    )

    return [passthrough_loader]


def generate_launch_description():

    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    add_launch_arg("base_frame", "base_link")
    add_launch_arg("use_multithread", "False")
    add_launch_arg("use_intra_process", "False")
    add_launch_arg("pointcloud_container_name", "pointcloud_container")
    add_launch_arg("use_concat_filter", "true")  # Use this to enable/disable the relay

    set_container_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container",
        condition=UnlessCondition(LaunchConfiguration("use_multithread")),
    )

    set_container_mt_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container_mt",
        condition=IfCondition(LaunchConfiguration("use_multithread")),
    )

    return launch.LaunchDescription(
        launch_arguments
        + [set_container_executable, set_container_mt_executable]
        + [OpaqueFunction(function=launch_setup)]
    )
