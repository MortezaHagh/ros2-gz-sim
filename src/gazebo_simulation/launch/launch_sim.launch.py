import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ur gazebo_simulation pkg #
    pkg_name = "gazebo_simulation"
    pkg_dir = get_package_share_directory(pkg_name)

    # config #
    use_sim_time_config = LaunchConfiguration("use_sim_time")
    declare_use_sim_time_config = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time")

    # Launch world #
    py_jt_sub = PathJoinSubstitution([pkg_dir, "launch", "launch_world.launch.py"])
    py_launch_des_sc = PythonLaunchDescriptionSource(py_jt_sub)
    gz_sim_world = IncludeLaunchDescription(py_launch_des_sc)

    # spawn robot #
    py_jt_sub = PathJoinSubstitution([pkg_dir, "launch", "spawn_robot.launch.py"])
    py_launch_des_sc = PythonLaunchDescriptionSource(py_jt_sub)
    spawn_robot = IncludeLaunchDescription(
        py_launch_des_sc,
        launch_arguments={"use_sim_time": use_sim_time_config}.items()
    )

    # Load RViz Configuration File #
    rviz_config_file = "config.rviz"
    rviz_config_path = os.path.join(pkg_dir, "rviz", rviz_config_file)

    # RViz2 Launch Configuration (RViz) #
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_node',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time_config}],
        arguments=['-d', rviz_config_path],
    )

    #
    sim_time_param = SetParameter(name="use_sim_time", value=use_sim_time_config)

    # LaunchDescription
    ld = LaunchDescription(
        [
            declare_use_sim_time_config,
            sim_time_param,
            gz_sim_world,
            spawn_robot,
            rviz_node
        ]
    )
    return ld
