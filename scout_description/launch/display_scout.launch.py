import os
import launch
import launch_ros

from ament_index_python.packages import get_package_share_directory

from math import pi

from launch.conditions import IfCondition, UnlessCondition
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable, PathJoinSubstitution
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)


def generate_launch_description():
    model_name = 'scout2.urdf.xacro'
    model_path = os.path.join(get_package_share_directory('scout_description'), "urdf", model_name)
   
    print(model_path)


    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution(
            [FindPackageShare("scout_description"), "urdf", model_name]
        ),
    ])

    robot_description = {"robot_description": robot_description_content}


    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("scout_description"), "rviz", "model_display.rviz"]
    )
   

    rviz_node = Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
                # arguments=["-d", rviz_config_file],
            )
    


    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters= [{"use_sim_time": True}, robot_description], #[{"use_sim_time": True}, robot_description],
      
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}],
    )

    pointcloud_to_laserscan_node = Node(
		package='pointcloud_to_laserscan',
		executable='pointcloud_to_laserscan_node',
		name='pointcloud_to_laserscan_node',
		remappings=[('cloud_in', "/ray/pointcloud2"),
					('scan', "/scan")],
		parameters=[{
			'transform_tolerance': 0.05,
			'min_height': 0.0,
			'max_height': 1.0,
			'angle_min': -pi,
			'angle_max': pi,
			'angle_increment': pi / 180.0 / 2.0,
			'scan_time': 1/10, # 10Hz
			'range_min': 0.1,
			'range_max': 100.0,
			'use_inf': True,
		}],
	)


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]
        ),
    )

    # Spawn robot
    gazebo_spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_scout",
        arguments=["-entity", "scout2", 
                   "-topic", "robot_description"],
        output="screen",
    )


    return launch.LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
            description='Use simulation clock if true'),

        launch.actions.LogInfo(msg='use_sim_time: '),
        launch.actions.LogInfo(msg=launch.substitutions.LaunchConfiguration('use_sim_time')),

            robot_state_publisher_node,
            joint_state_publisher_node,
            rviz_node,
            gazebo,
            gazebo_spawn_robot,
            pointcloud_to_laserscan_node,

    ])
