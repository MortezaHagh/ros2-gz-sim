import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
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

    # load urdf file #
    urdf_file = "uvc_robot.xacro"
    urdf_file_path = os.path.join(pkg_dir, "urdf", urdf_file)

    #
    nodes = []

    for i in range(1, 3):
        robot_ns = "r" + str(i)

        # # static_transform_publisher #
        # tf_static = Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name=f'static_transform_publisher_{robot_ns}',
        #     output='screen',
        #     arguments=['0', '0', '0', '0', '0', '0', 'map', robot_ns+'/odom'],
        # )

        # robot state publisher #
        robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name=f'robot_state_publisher',
            output="screen",
            namespace=robot_ns,
            emulate_tty=True,
            parameters=[{'frame_prefix': f'{robot_ns}/', 'use_sim_time': use_sim_time_config,
                        'robot_description': Command(['xacro ', urdf_file_path, ' robot_name:=', robot_ns])}]
        )

        # robot state publisher #
        joint_state_publisher_node = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name=f'joint_state_publisher',
            output='screen',
            namespace=robot_ns,
            parameters=[{'use_sim_time': use_sim_time_config}],
        )

        # spawn robot #
        y = "0.0"
        if robot_ns == "r2":
            y = "2.0"
        gz_spawn_node = Node(
            package="ros_gz_sim",
            executable="create",
            name=f"robot_spawn",
            namespace=robot_ns,
            parameters=[{'use_sim_time': use_sim_time_config}],
            arguments=[
                "-name", "uvc_robot"+robot_ns,
                "-allow_renaming", "true",
                "-topic", "robot_description",
                "-x", "0.0",
                "-y", y,
                "-z", "0.5",
            ],
            output="screen",
        )

        # ROS-Gazebo Bridge #
        file_name = "gz_bridge_ns_" + robot_ns + ".yaml"
        bridge_params = os.path.join(pkg_dir, "params", file_name)
        gz_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name=f"gz_bridge{robot_ns}",
            arguments=['--ros-args', '-p', f'config_file:={bridge_params}'],
            # remappings=[# there are no remappings for this robot description],
            output="screen",
        )

        #
        # nodes.append(tf_static)
        nodes.append(robot_state_publisher_node)
        # nodes.append(joint_state_publisher_node)
        nodes.append(gz_spawn_node)
        nodes.append(gz_bridge)
    # end for loop #

    #
    return LaunchDescription(nodes)
