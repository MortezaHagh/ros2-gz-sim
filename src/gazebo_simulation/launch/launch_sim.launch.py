import os
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription  # , DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution  # , LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # ur simulation pkg #
    pkg_name = "gazebo_simulation"
    pkg_dir = get_package_share_directory(pkg_name)

    # Launch world #
    pjs = PathJoinSubstitution([pkg_dir, "launch", "launch_world.launch.py"])
    plds = PythonLaunchDescriptionSource(pjs)
    gz_sim = IncludeLaunchDescription(plds)

    # spawn robot #
    pjs = PathJoinSubstitution([pkg_dir, "launch", "spawn_robot.launch.py"])
    plds = PythonLaunchDescriptionSource(pjs)
    spawn_sim = IncludeLaunchDescription(plds)

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
            spawn_sim,
            rviz_node
        ]
    )
    return ld
