import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ur simulation pkg #
    pkg_name = "gazebo_simulation"
    pkg_dir = get_package_share_directory(pkg_name)

    # load world file #
    # world_file_path = "empty.sdf"
    world_file = "slide_1.world"
    world_file_path = os.path.join(pkg_dir, "worlds", world_file)
    print("world_file_path", world_file_path)
    world_config = LaunchConfiguration("world")
    declare_world_arg = DeclareLaunchArgument("world",
                                              default_value=[
                                                  "-r ", world_file_path],
                                              description="SDF World File")

    # Declare GazeboSim Launch #
    # gzsim_pkg = FindPackageShare("ros_gz_sim")
    gzsim_pkg = get_package_share_directory("ros_gz_sim")
    pjs = PathJoinSubstitution([gzsim_pkg, "launch", "gz_sim.launch.py"])
    plds = PythonLaunchDescriptionSource(pjs)
    gz_sim = IncludeLaunchDescription(
        plds,
        launch_arguments={"gz_args": world_config}.items()
    )

    # LaunchDescription
    ld = LaunchDescription(
        [
            declare_world_arg,
            gz_sim
        ]
    )
    return ld
