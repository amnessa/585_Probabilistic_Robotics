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

        # TF and simulation clock (GZ -> ROS)
        '/model/vehicle_green/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
        '/model/vehicle_blue/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
        '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
    ]
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridge_args,
        output='screen',
        # Remap the ROS-side names of the TF topics to /tf
        remappings=[
            ('/model/vehicle_green/tf', '/tf'),
            ('/model/vehicle_blue/tf', '/tf'),
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

    return LaunchDescription([gz, bridge, cam_tf, rviz, tf_blue, tf_green])