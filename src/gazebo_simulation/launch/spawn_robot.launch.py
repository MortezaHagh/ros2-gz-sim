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

    # load world file #
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

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # robot state publisher #
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
    ign_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        arguments=[
            "/clock" + "@rosgraph_msgs/msg/Clock" + "[gz.msgs.Clock",
            "/cmd_vel" + "@geometry_msgs/msg/Twist" + "@gz.msgs.Twist",
            "/tf" + "@tf2_msgs/msg/TFMessage" + "[gz.msgs.Pose_V",
            "/odom" + "@nav_msgs/msg/Odometry" + "[gz.msgs.Odometry",
            "/velodyne_laserscan" + "@sensor_msgs/msg/LaserScan" +
                "[gz.msgs.LaserScan",
            "/imu" + "@sensor_msgs/msg/Imu" + "[gz.msgs.IMU",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/usb_cam/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/usb_cam/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/usb_cam/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"
        ],
        remappings=[
            # there are no remappings for this robot description
        ],
        output="screen",
    )

    # parametes #
    sim_time_param = SetParameter(name="use_sim_time", value=True)

    #
    tf_static = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '1.5', '0', '0', '0', 'base_link',
                   'uvc_robot/velodyne_laserscan/head_hokuyo_sensor'],
    )

    #
    return LaunchDescription([
        sim_time_param,
        robot_state_publisher_node,
        joint_state_publisher_node,
        # tf_static,
        gz_spawn_node,
        ign_bridge,
    ])
