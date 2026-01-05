#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
	DeclareLaunchArgument,
	OpaqueFunction,
)
from launch.substitutions import (
	LaunchConfiguration,
)
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):

	return [
		Node(
			package='routine',
			executable='routine_node',
			name='routine_node',
			respawn = True,
		),
	]

def generate_launch_description():
	ld = LaunchDescription([
		OpaqueFunction(function = launch_setup)
	])

	return ld