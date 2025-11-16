from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import shutil

def generate_launch_description():
    pkg = FindPackageShare('homework1_launcher')
    world_path = PathJoinSubstitution([pkg, 'worlds', 'diff_drive_hw1.sdf'])
    rviz_cfg = PathJoinSubstitution([pkg, 'rviz', 'diff_drive.rviz'])

    gz_cmd = ['gz', 'sim', '-r', world_path] if shutil.which('gz') \
        else ['ign', 'gazebo', '-r', world_path, '--force-version', '6']
    gz = ExecuteProcess(cmd=gz_cmd, output='screen')

    bridge_args = [
        # cmd_vel (ROS -> GZ) and odom (GZ -> ROS)
        '/model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
        '/model/vehicle_green/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
        '/model/vehicle_blue/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
        '/model/vehicle_green/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',

        # Camera (GZ -> ROS)
        '/camera/image@sensor_msgs/msg/Image@ignition.msgs.Image',
        '/camera/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image',
        '/camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',

        # Lidar + IMU on blue robot (GZ -> ROS)
        '/lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
        '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',

        # Ground-truth poses (GZ -> ROS). Bridge Pose_V to TFMessage on per-model topics.
        '/model/vehicle_blue/pose@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
        '/model/vehicle_green/pose@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
        '/model/vehicle_blue/pose_static@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
        '/model/vehicle_green/pose_static@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',

        # Clock (GZ -> ROS)
        '/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock',
    ]
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridge_args,
        output='screen',
    )

    # One broadcaster per robot, remapped to the bridged topics
    pose_tf_blue = Node(
        package='homework1_launcher',
        executable='pose_tf_broadcaster',
        name='pose_tf_broadcaster_blue',
        output='screen',
        remappings=[
            ('pose', '/model/vehicle_blue/pose'),
            ('pose_static', '/model/vehicle_blue/pose_static'),
        ]
    )

    # Static TF from chassis -> camera sensor frame (matches SDF pose: 0.30 0 0.40)
    cam_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.30', '0', '0.40', '0', '0', '0',
                   'vehicle_green/chassis', 'vehicle_green/chassis/rgbd'],
        output='screen'
    )

    pose_tf_green = Node(
        package='homework1_launcher',
        executable='pose_tf_broadcaster',
        name='pose_tf_broadcaster_green',
        output='screen',
        remappings=[
            ('pose', '/model/vehicle_green/pose'),
            ('pose_static', '/model/vehicle_green/pose_static'),
        ]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    tf_blue = Node(package='tf2_ros', executable='static_transform_publisher',
                   arguments=['0', '2', '0', '0', '0', '0', 'world', 'vehicle_blue/odom'],
                   output='screen')
    tf_green = Node(package='tf2_ros', executable='static_transform_publisher',
                    arguments=['0', '-2', '0', '0', '0', '0', 'world', 'vehicle_green/odom'],
                    output='screen')

    # chassis -> lidar_link (matches SDF pose 0.6 0 0.6)
    lidar_mount_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.6','0','0.6','0','0','0',
                   'vehicle_blue/chassis','vehicle_blue/lidar_link'],
        output='screen'
    )
    # lidar_link -> gpu_lidar (identity; matches LaserScan header.frame_id)
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0',
                   'vehicle_blue/lidar_link','vehicle_blue/lidar_link/gpu_lidar'],
        output='screen'
    )

    # chassis -> imu_link (matches SDF pose 0 0 0.3)
    imu_mount_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0.3','0','0','0',
                   'vehicle_blue/chassis','vehicle_blue/imu_link'],
        output='screen'
    )
    # imu_link -> imu_sensor (identity)
    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0',
                   'vehicle_blue/imu_link','vehicle_blue/imu_link/imu_sensor'],
        output='screen'
    )

    return LaunchDescription([gz, bridge, pose_tf_blue, pose_tf_green,
                              tf_blue, tf_green, cam_tf, rviz,
                              lidar_mount_tf, lidar_tf, imu_mount_tf, imu_tf])