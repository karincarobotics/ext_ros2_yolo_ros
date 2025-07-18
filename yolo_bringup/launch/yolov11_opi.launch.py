# Copyright (C) 2024 Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    # Create the launch description and populate
    ld = LaunchDescription()

    namespace = LaunchConfiguration('namespace')
    declare_namespace_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='Top-level namespace')
    ld.add_action(declare_namespace_cmd)
    bringup_dir = get_package_share_directory('yolo_bringup')

    x = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bringup_dir,
                "launch",
                "yolo.launch.py",
            )
        ),
        launch_arguments={
            "classes": LaunchConfiguration("classes", default='"0"'),  #track only persons
            "model": LaunchConfiguration("model", default=os.path.join(bringup_dir, "models", "yolo11n_rknn_model")),
            "tracker": LaunchConfiguration("tracker", default="bytetrack.yaml"),
            "device": LaunchConfiguration("device", default="cpu"),
            "enable": LaunchConfiguration("enable", default="True"),
            "threshold": LaunchConfiguration("threshold", default="0.75"),
            "use_debug": LaunchConfiguration("use_debug", default="False"),
            "imgsz_width": LaunchConfiguration("imgsz_width", default="640"),
            "imgsz_height": LaunchConfiguration("imgsz_height", default="640"),
            "use_tracking": LaunchConfiguration("use_tracking", default="False"),
            "half": LaunchConfiguration("half", default="True"),
            "input_image_topic": LaunchConfiguration(
                "input_image_topic", default="/camera_turret/color/image_raw"
            ),
            "image_reliability": LaunchConfiguration(
                "image_reliability", default="1"
            ),
            "namespace": namespace
        }.items(),
    )

    ld.add_action(x)

    #Launch debugger separately
    #ros2 run yolo_ros debug_node --ros-args -p image_reliability:=1 --remap image_raw:=/camera_turret/color/image_raw --remap detections:=/tracking


    return ld
