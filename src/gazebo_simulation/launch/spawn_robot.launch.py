import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # robot description pkg #
    pkg_name = "robot_description"
    pkg_dir = get_package_share_directory(pkg_name)

    # sim time config #
    use_sim_time_config = LaunchConfiguration("use_sim_time")
    declare_use_sim_time_config = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time")

    # Log message if simulation time is enabled
    log_sim_time = LogInfo(
        condition=IfCondition(use_sim_time_config),
        msg="Simulation time is enabled."
    )

    # if 'GAZEBO_MODEL_PATH' in os.environ:
    #     os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share' + ':' + gazebo_models_path
    # else:
    #     os.environ['GAZEBO_MODEL_PATH'] =  install_dir + "/share" + ':' + gazebo_models_path

    # if 'GAZEBO_PLUGIN_PATH' in os.environ:
    #     os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    # else:
    #     os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    # load urdf file #
    urdf_file = "uvc_robot.xacro"
    urdf_file_path = os.path.join(pkg_dir, "urdf", urdf_file)
    print("urdf_file_path", urdf_file_path)

    # robot state publisher #
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        output="screen",
        emulate_tty=True,
        parameters=[{
            'use_sim_time': use_sim_time_config,
            'robot_description': Command(['xacro ', urdf_file_path])}]
    )

    # robot state publisher #
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time_config}],
    )

    # spawn robot #
    gz_spawn_node = Node(
        package="ros_gz_sim",
        executable="create",
        name="uvc_robot_spawn",
        parameters=[{'use_sim_time': use_sim_time_config}],
        arguments=[
            "-name", "uvc_robot",
            "-allow_renaming", "true",
            "-topic", "robot_description",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.5",
        ],
        output="screen",
    )

    # ROS-Gazebo Bridge #
    bridge_params = os.path.join(pkg_dir, "params", "gz_bridge.yaml")
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        parameters=[{
            'config_file': bridge_params,
            'use_sim_time': True,
        }],
        output="screen",
    )

    #
    return LaunchDescription([
        declare_use_sim_time_config,
        log_sim_time,
        robot_state_publisher_node,
        joint_state_publisher_node,
        gz_spawn_node,
        gz_bridge,
    ])
