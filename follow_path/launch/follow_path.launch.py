from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    map_path_arg = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Path to the map file for navigation'
    )

    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='true',
        description='Enable simulation mode'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Enable RViz visualization'
    )

    path_csv_arg = DeclareLaunchArgument(
        'path_csv',
        default_value='path.csv',
        description='CSV file containing path coordinates'
    )

    home_csv_arg = DeclareLaunchArgument(
        'home_csv',
        default_value='home.csv',
        description='CSV file containing home position'
    )

    cycles_arg = DeclareLaunchArgument(
        'cycles',
        default_value='1',
        description='Number of navigation cycles to perform'
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('linorobot2_navigation'),
                'launch',
                'navigation.launch.py'
            ])
        ]),
        launch_arguments={
            'sim': LaunchConfiguration('sim'),
            'rviz': LaunchConfiguration('rviz'),
            'map': LaunchConfiguration('map')
        }.items()
    )

    follow_path_node = Node(
        package='follow_path',
        executable='follow_path',
        name='follow_path',
        arguments=[
            LaunchConfiguration('path_csv'),
            LaunchConfiguration('home_csv'),
            LaunchConfiguration('cycles')
        ],
        output='screen'
    )

    return LaunchDescription([
        map_path_arg,
        sim_arg,
        rviz_arg,
        path_csv_arg,
        home_csv_arg,
        cycles_arg,
        navigation_launch,
        follow_path_node
    ])
