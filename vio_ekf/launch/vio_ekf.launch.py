import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_vio = get_package_share_directory('vio_ekf')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # 1. Start Ignition Gazebo with our world
    sdf_path = os.path.join(pkg_vio, 'worlds', 'landmarks.sdf')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {sdf_path}'}.items(),
    )

    # 2. Bridge ROS2 <-> Ignition
    # Bridges:
    #   /imu (Ignition) -> /imu (ROS2)
    #   /camera/image_raw (Ignition) -> /camera/image_raw (ROS2)
    #   /model/vio_robot/pose (Ignition) -> TFMessage for pose_tf_broadcaster
    #   /clock for sim time synchronization
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # IMU (GZ -> ROS)
            '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
            # Camera (GZ -> ROS)
            '/camera/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
            # Ground-truth pose as TFMessage (GZ -> ROS) - matches pose_tf_broadcaster expectations
            '/model/vio_robot/pose@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            '/model/vio_robot/pose_static@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            # Clock (GZ -> ROS) - required for use_sim_time
            '/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock',
        ],
        output='screen'
    )

    # 3. Ground Truth Publisher (Our Custom Node)
    # Subscribes to TFMessage on 'pose' and 'pose_static' topics and broadcasts to /tf
    # Use remappings to connect to the bridged topics
    ground_truth = Node(
        package='vio_ekf',
        executable='pose_tf_broadcaster',
        name='pose_tf_broadcaster',
        output='screen',
        remappings=[
            ('pose', '/model/vio_robot/pose'),
            ('pose_static', '/model/vio_robot/pose_static'),
        ]
    )

    # 4. Rviz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        # arguments=['-d', os.path.join(pkg_vio, 'rviz', 'vio.rviz')], # Enable once we save a config
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        bridge,
        ground_truth,
        rviz
    ])