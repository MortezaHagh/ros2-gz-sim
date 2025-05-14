import os
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # robot description pkg #
    pkg_name = "robot_description"
    pkg_dir = get_package_share_directory(pkg_name)

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
        parameters=[{'use_sim_time': True,
                     'robot_description': Command(['xacro ', urdf_file_path])}]
    )

    # robot state publisher #
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # spawn robot #
    gz_spawn_node = Node(
        package="ros_gz_sim",
        executable="create",
        name="uvc_robot_spawn",
        parameters=[{'use_sim_time': True}],
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
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        remappings=[
            # there are no remappings for this robot description
        ],
        output="screen",
    )

    # parametes #
    sim_time_param = SetParameter(name="use_sim_time", value=True)

    #
    return LaunchDescription([
        sim_time_param,
        robot_state_publisher_node,
        joint_state_publisher_node,
        gz_spawn_node,
        gz_bridge,
    ])
