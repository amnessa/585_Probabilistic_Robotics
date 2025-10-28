from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import shutil

def generate_launch_description():
    pkg = FindPackageShare('homework1_launcher')
    # use the new world
    world_path = PathJoinSubstitution([pkg, 'worlds', 'diff_drive_hw1.sdf'])
    rviz_cfg = PathJoinSubstitution([pkg, 'rviz', 'diff_drive.rviz'])

    # Prefer gz if available, else Fortress ign
    gz_cmd = ['gz', 'sim', '-r', world_path] if shutil.which('gz') \
        else ['ign', 'gazebo', '-r', world_path, '--force-version', '6']
    gz = ExecuteProcess(cmd=gz_cmd, output='screen')

    # Humble parameter_bridge: pass mappings on CLI (no YAML)
    bridge_args = [
        '/model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
        '/model/vehicle_green/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
        '/model/vehicle_blue/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
        '/model/vehicle_green/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
    ]
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                  arguments=bridge_args, output='screen')

    rviz = Node(package='rviz2', executable='rviz2',
                arguments=['-d', rviz_cfg], output='screen')

    tf_blue = Node(package='tf2_ros', executable='static_transform_publisher',
                   arguments=['0', '2', '0', '0', '0', '0', 'world', 'vehicle_blue/odom'],
                   output='screen')
    tf_green = Node(package='tf2_ros', executable='static_transform_publisher',
                    arguments=['0', '-2', '0', '0', '0', '0', 'world', 'vehicle_green/odom'],
                    output='screen')

    return LaunchDescription([gz, bridge, rviz, tf_blue, tf_green])