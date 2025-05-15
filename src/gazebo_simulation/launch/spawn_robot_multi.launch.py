import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, GroupAction
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, PushRosNamespace
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
    urdf_file = "uvc_robot_m.xacro"
    urdf_file_path = os.path.join(pkg_dir, "urdf", urdf_file)

    #
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_config)

    # Log message if simulation time is enabled
    log_sim_time = LogInfo(
        condition=IfCondition(use_sim_time_config),
        msg="Simulation time is enabled."
    )
    ld.add_action(log_sim_time)

    # Log message if simulation time is disabled
    log_sim_time_disabled = LogInfo(
        condition=UnlessCondition(use_sim_time_config),
        msg="Simulation time is disabled."
    )
    ld.add_action(log_sim_time_disabled)

    #
    for i in range(1, 3):
        # g = [PushRosNamespace(robot_ns)]
        g = []
        robot_ns = "r" + str(i)
        y = str((i-1)*2.0)

        # Log message for spawning robot
        log_robot_spawn = LogInfo(
            msg=f"Spawning robot: {robot_ns}"
        )

        tf_static = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'static_transform_publisher_{robot_ns}',
            output='screen',
            arguments=['0', y, '0', '0', '0', '0', 'map', robot_ns+'/odom'],
        )

        # robot state publisher #
        robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output="screen",
            namespace=robot_ns,
            emulate_tty=True,
            parameters=[{
                'frame_prefix': f'{robot_ns}/',
                'use_sim_time': use_sim_time_config,
                'robot_description': Command(['xacro ', urdf_file_path, ' namespace:=', robot_ns])}],
            # remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
        )

        # robot state publisher #
        joint_state_publisher_node = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            namespace=robot_ns,
            parameters=[{'use_sim_time': use_sim_time_config}],
        )

        # spawn robot #
        gz_spawn_node = Node(
            package="ros_gz_sim",
            executable="create",
            name="robot_spawn",
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
        file_name = "gz_bridge.yaml"
        bridge_params = os.path.join(pkg_dir, "params", file_name)
        gz_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            namespace=robot_ns,
            name=f"gz_bridge{robot_ns}",
            parameters=[{
                    'config_file': bridge_params,
                    'expand_gz_topic_names': True,
                    'use_sim_time': use_sim_time_config}],
            remappings=[(f'/{robot_ns}/tf', '/tf')],
            output="screen",
        )

        #
        g.append(tf_static)
        g.append(log_robot_spawn)
        g.append(robot_state_publisher_node)
        # g.append(joint_state_publisher_node)
        g.append(gz_spawn_node)
        g.append(gz_bridge)
        group = GroupAction(g)
        ld.add_action(group)

    # end for loop #

    #
    return ld
