import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, GroupAction
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions.find_executable import FindExecutable
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

        # Log message for spawning robot
        log_robot_spawn = LogInfo(
            msg=f"Spawning robot: {robot_ns}"
        )

        # robot state publisher #
        robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name=f'robot_state_publisher',
            output="screen",
            namespace=robot_ns,
            emulate_tty=True,
            parameters=[{
                'frame_prefix': f'{robot_ns}/',
                'use_sim_time': use_sim_time_config,
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
                "-name", "uvc_robot_"+robot_ns,
                '-string', Command([FindExecutable(name='xacro'), ' ', 'namespace:=', robot_ns, ' ', urdf_file_path]),
                # "-allow_renaming", "true",
                # "-topic", "robot_description",
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
            parameters=[{
                    'config_file': bridge_params,
                    'expand_gz_topic_names': True,
                    'use_sim_time': use_sim_time_config}],
            output="screen",
        )

        #
        # g.append(tf_static)
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
