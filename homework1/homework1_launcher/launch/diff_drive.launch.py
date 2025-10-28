from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # Paths to assets in this package
    world_path = PathJoinSubstitution([
        FindPackageShare('homework1_launcher'), 'worlds', 'diff_drive.sdf'
    ])
    bridge_yaml = PathJoinSubstitution([
        FindPackageShare('homework1_launcher'), 'config', 'diff_drive.yaml'
    ])
    rviz_cfg = PathJoinSubstitution([
        FindPackageShare('homework1_launcher'), 'rviz', 'diff_drive.rviz'
    ])

    # Start Gazebo Sim with the diff_drive world
    gz = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        output='screen'
    )

    # Start the ROS <-> Gazebo bridge using the YAML config
    bridge = Node(
        package='ros_gz_bridge',
        executable='ros_gz_bridge',
        arguments=['-c', bridge_yaml],
        output='screen'
    )

    # Start RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        output='screen'
    )

    # Static TFs (world -> odom frames), standard CLI expects 8 args: x y z yaw pitch roll frame child
    tf_blue = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '2', '0', '0', '0', '0', 'world', 'vehicle_blue/odom'],
        output='screen'
    )
    tf_green = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '-2', '0', '0', '0', '0', 'world', 'vehicle_green/odom'],
        output='screen'
    )

    return LaunchDescription([gz, bridge, rviz, tf_blue, tf_green])