import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ur gazebo_simulation pkg #
    pkg_name = "gazebo_simulation"
    pkg_dir = get_package_share_directory(pkg_name)

    # load world file #
    # world_file = "empty.world.sdf"
    world_file = "test2.world"
    world_file_path = os.path.join(pkg_dir, "worlds", world_file)
    world_config = LaunchConfiguration("world")
    declare_world_arg = DeclareLaunchArgument(
        "world",
        default_value=["-r ", world_file_path],
        description="SDF World File")

    # Declare GazeboSim Launch #
    # gzsim_pkg = FindPackageShare("ros_gz_sim")
    gzsim_pkg = get_package_share_directory("ros_gz_sim")
    py_jt_sub = PathJoinSubstitution([gzsim_pkg, "launch", "gz_sim.launch.py"])
    py_launch_des_sc = PythonLaunchDescriptionSource(py_jt_sub)
    gz_sim = IncludeLaunchDescription(py_launch_des_sc, launch_arguments={"gz_args": world_config}.items())

    # LaunchDescription
    ld = LaunchDescription(
        [
            declare_world_arg,
            gz_sim
        ]
    )
    return ld
