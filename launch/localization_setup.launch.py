from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

ARGUMENTS = [
    DeclareLaunchArgument('rviz', default_value='true',
                          choices=['true', 'false'], description='Launch RViz'),
    DeclareLaunchArgument('world', default_value='maze',
                          description='World to load'),
    DeclareLaunchArgument('x', default_value='-9.0',
                          description='Initial x position'),
    DeclareLaunchArgument('y', default_value='8.0',
                          description='Initial y position'),
    DeclareLaunchArgument('yaw', default_value='3.14',
                          description='Initial yaw rotation')
]

def generate_launch_description():
    turtlebot4_ignition_package = get_package_share_directory('turtlebot4_ignition_bringup')

    ignition_launch = PathJoinSubstitution([turtlebot4_ignition_package, 'launch', 'ignition.launch.py'])
    robot_spawn = PathJoinSubstitution([turtlebot4_ignition_package, 'launch', 'turtlebot4_spawn.launch.py'])

    ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ignition_launch]),
        launch_arguments=[
            ('world', LaunchConfiguration('world'))
        ]
    )
    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_spawn]),
        launch_arguments=[
            ('rviz', LaunchConfiguration('rviz')),
            ('x', LaunchConfiguration('x')),
            ('y', LaunchConfiguration('y')),
            ('yaw', LaunchConfiguration('yaw')),
            ('localization', 'true'),
            ('map', '/opt/ros/humble/share/turtlebot4_navigation/maps/maze.yaml')
        ]
    )

    initial_pose = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='turtlebot_test',
                executable='initial_pose_publisher',
                name='initial_pose_node',
                output='screen',
                parameters=[{
                    'x': LaunchConfiguration('x'),
                    'y': LaunchConfiguration('y'),
                    'yaw': LaunchConfiguration('yaw'),
                    'use_sim_time': True
                    }]
            )
        ]
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(ignition)
    ld.add_action(spawn)
    ld.add_action(initial_pose)
    return ld