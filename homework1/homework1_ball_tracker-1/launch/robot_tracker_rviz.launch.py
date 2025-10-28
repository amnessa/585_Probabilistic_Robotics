from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='homework1_ball_tracker',
            executable='ball_tracker_node',
            name='ball_tracker_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            remappings=[
                ('/rsd455_img', '/camera/image_raw'),
                ('/rsd455_depth', '/camera/depth'),
                ('/ball_position', '/ball_position'),
                ('/target_pixel_coords', '/target_pixel_coords')
            ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            parameters=[{'config': '/homework1_ball_tracker/rviz/ball_tracker.rviz'}]
        )
    ])