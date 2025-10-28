from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    pkg = FindPackageShare('homework1_ball_tracker')
    params = PathJoinSubstitution([pkg, 'config', 'params.yaml'])
    rviz_cfg = PathJoinSubstitution([pkg, 'rviz', 'camera_tracker.rviz'])

    return LaunchDescription([
        # Ball tracker (uses image + depth only)
        Node(
            package='homework1_ball_tracker',
            executable='camera_tracker_node',   # renamed to match the built target
            name='camera_tracker_node',
            output='screen',
            parameters=[params],
            remappings=[
                ('/rsd455_img', '/camera/image'),
                ('/rsd455_depth', '/camera/depth_image'),
                ('/ball_position', '/ball_position'),
                ('/target_pixel_coords', '/target_pixel_coords'),
            ],
        ),

        # Simple follower for the GREEN robot (subscribes target_pixel_coords)
        Node(
            package='homework1_ball_tracker',
            executable='green_follower_node',
            name='green_follower',
            output='screen',
            parameters=[{
                'image_width': 640,
                'goal_dist': 1.5,
                'k_ang': 1.2,         # rad/s for full-width error
                'k_lin': 0.8,         # m/s per meter of depth error
                'max_lin': 0.8,
                'max_ang': 1.2,
                'search_omega': 0.6,  # rotate to search when target lost
                'lost_timeout': 0.5,  # seconds without detections => search
            }],
            remappings=[
                ('/target_pixel_coords', '/target_pixel_coords'),
                ('/cmd_vel_out', '/model/vehicle_green/cmd_vel'),
            ],
        ),

        # Image preview (optional)
        Node(
            package='image_view',
            executable='image_view',
            name='image_view',
            output='screen',
            remappings=[('/image', '/camera/image')],
        ),

        # RViz with camera and overlays
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_cfg],
        ),
    ])