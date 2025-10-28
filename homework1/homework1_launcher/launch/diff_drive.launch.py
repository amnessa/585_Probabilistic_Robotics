from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import shutil

def generate_launch_description():
    pkg_share = FindPackageShare('homework1_launcher')
    world_path = PathJoinSubstitution([pkg_share, 'worlds', 'vehicle.sdf'])
    bridge_yaml = PathJoinSubstitution([pkg_share, 'config', 'diff_drive.yaml'])
    rviz_cfg = PathJoinSubstitution([pkg_share, 'rviz', 'diff_drive.rviz'])

    # Prefer gz if available; else fallback to ign (Fortress)
    if shutil.which('gz'):
        gz_cmd = ['gz', 'sim', '-r', world_path]
    elif shutil.which('ign'):
        gz_cmd = ['ign', 'gazebo', '-r', world_path, '--force-version', '6']
    else:
        # Will fail clearly if neither is available
        gz_cmd = ['gz', 'sim', '-r', world_path]

    gz = ExecuteProcess(cmd=gz_cmd, output='screen')

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['-c', bridge_yaml],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        output='screen'
    )

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