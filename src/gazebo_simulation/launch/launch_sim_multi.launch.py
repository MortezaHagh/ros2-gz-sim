import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ur simulation pkg #
    pkg_name = "gazebo_simulation"
    pkg_dir = get_package_share_directory(pkg_name)

    # Launch world #
    pjs = PathJoinSubstitution([pkg_dir, "launch", "launch_world.launch.py"])
    plds = PythonLaunchDescriptionSource(pjs)
    gz_sim = IncludeLaunchDescription(plds)

    # spawn robot #
    pjs = PathJoinSubstitution([pkg_dir, "launch", "spawn_robot_multi.launch.py"])
    plds = PythonLaunchDescriptionSource(pjs)
    spawn_multi = IncludeLaunchDescription(plds)
    # PushROSNamespace('turtlesim2'),

    # Load RViz Configuration File #
    rviz_config_file = "config.rviz"
    rviz_config_path = os.path.join(pkg_dir, "rviz", rviz_config_file)
    print("RViz Config Loaded !")

    # RViz2 Launch Configuration (RViz) #
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_node',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_path],
    )

    # LaunchDescription
    ld = LaunchDescription(
        [
            gz_sim,
            spawn_multi,
            rviz_node
        ]
    )
    return ld
