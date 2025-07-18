import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import RegisterEventHandler, LogInfo
from launch.conditions import LaunchConfigurationEquals
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Create the launch description and populate
    ld = LaunchDescription()
    namespace = LaunchConfiguration('namespace')
    declare_namespace_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='Top-level namespace')
    ld.add_action(declare_namespace_cmd)

    camera_front_node = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace=namespace,
        name="camera_turret",
        parameters=[{"initial_reset": True,
                     "pointcloud.enable": False,
                     "accelerate_gpu_with_glsl": False,
                     "spatial_filter.enable": False,
                     "temporal_filter.enable": False,
                     "rgb_camera.color_profile": "640x480x60",
                     "depth_module.infra_profile": "640x480x60",
                     "depth_module.depth_profile": "640x480x60",
                     "camera_name": "camera_turret",
                     "enable_color": True,
                     "enable_depth": False,
                     "enable_infra1": False,
                     "enable_infra2": False,
                     "clip_distance": 2.5
                     }]
    )
    ld.add_action(camera_front_node)
    return ld