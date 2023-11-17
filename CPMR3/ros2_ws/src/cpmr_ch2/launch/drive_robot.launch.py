import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('goal_x', default_value = '0.0', description = 'goal (x)'),
        DeclareLaunchArgument('goal_y', default_value = '0.0', description = 'goal (y)'),
        DeclareLaunchArgument('goal_t', default_value = '0.0', description = 'goal (t)'),
        DeclareLaunchArgument('max_vel', default_value = '0.0', description = 'maxium Velocity '),
        DeclareLaunchArgument('max_gain', default_value = '0.0', description = 'max Gain'),
        DeclareLaunchArgument('newGoal', default_value = "0.0&0.0", description = 'newGoal'),
        
        Node(
            package = 'cpmr_ch2',
            executable = 'drive_to_goal',
            name = 'drive_to_goal',
            parameters = [
                {'goal_x' : LaunchConfiguration('goal_x')},
                {'goal_y' : LaunchConfiguration('goal_y')},
                {'goal_t' : LaunchConfiguration('goal_t')},
                {'max_vel' : LaunchConfiguration('max_vel')},
                {'max_gain' : LaunchConfiguration('max_gain')},
                {'newGoal' : LaunchConfiguration('newGoal')},
            ],
        ),
    ])

