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
                          description='Initial yaw rotation'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'], description='Use simulation time'),
]

def generate_launch_description():
    turtlebot4_ignition_package = get_package_share_directory('turtlebot4_ignition_bringup')
    turtlebot4_navigation_package = get_package_share_directory('turtlebot4_navigation')

    ignition_launch = PathJoinSubstitution([turtlebot4_ignition_package, 'launch', 'turtlebot4_ignition.launch.py'])
    navigation_launch = PathJoinSubstitution([turtlebot4_navigation_package, 'launch', 'localization.launch.py'])

    ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ignition_launch]),
        launch_arguments=[
            ('world', LaunchConfiguration('world')),
            ('rviz', LaunchConfiguration('rviz')),
            ('x', LaunchConfiguration('x')),
            ('y', LaunchConfiguration('y')),
            ('yaw', LaunchConfiguration('yaw')),
        ]
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([navigation_launch]),
        launch_arguments=[
            ('map', PathJoinSubstitution([turtlebot4_navigation_package, 'maps', 'maze.yaml'])),
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
        ]
    )

    initial_pose = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='turtlebot_test',
                executable='initial_pose',
                name='initial_pose',
                output='screen',
                parameters=[{
                    'x': LaunchConfiguration('x'),
                    'y': LaunchConfiguration('y'),
                    'yaw': LaunchConfiguration('yaw'),
                    }]
            )
        ]
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(ignition)
    ld.add_action(navigation)
    ld.add_action(initial_pose)
    return ld