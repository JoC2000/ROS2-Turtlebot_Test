from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    turtlebot4_ignition_package = get_package_share_directory('turtlebot4_ignition_bringup')
    turtlebot4_navigation_package = get_package_share_directory('turtlebot4_navigation')

    ignition_launch = PathJoinSubstitution([turtlebot4_ignition_package, 'launch', 'turtlebot4_ignition.launch.py'])
    navigation_launch = PathJoinSubstitution([turtlebot4_navigation_package, 'launch', 'localization.launch.py'])

    ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ignition_launch]),
        launch_arguments=[
            ('world', 'maze'),
            ('rviz', 'true')
        ]
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([navigation_launch]),
        launch_arguments=[
            ('map', PathJoinSubstitution([turtlebot4_navigation_package, 'maps', 'maze.yaml'])),
        ]
    )

    ld = LaunchDescription()
    ld.add_action(ignition)
    ld.add_action(navigation)
    return ld