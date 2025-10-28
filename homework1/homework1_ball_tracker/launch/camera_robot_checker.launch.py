from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    pkg = FindPackageShare('homework1_ball_tracker')
    params = PathJoinSubstitution([pkg, 'config', 'params.yaml'])
    rviz_cfg = PathJoinSubstitution([pkg, 'rviz', 'camera_tracker.rviz'])

    return LaunchDescription([
        Node(
            package='homework1_ball_tracker',
            executable='camera_tracker_node',
            name='camera_tracker_node',
            output='screen',
            parameters=[params],
        ),

        Node(
            package='homework1_ball_tracker',
            executable='green_follower_node',
            name='green_follower',
            output='screen',
            parameters=[{
                'image_width': 640,
                'goal_dist': 1.5,
                'k_ang': 1.2,
                'k_lin': 0.8,
                'max_lin': 0.8,
                'max_ang': 1.2,
                'search_omega': 0.6,
                'lost_timeout': 0.5,
            }],
            remappings=[
                ('/target_pixel_coords', '/target_pixel_coords'),
                ('/cmd_vel_out', '/model/vehicle_green/cmd_vel'),
            ],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_cfg],
        ),
    ])