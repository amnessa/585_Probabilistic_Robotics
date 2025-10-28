from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='homework1_ball_tracker',
            executable='ball_tracker_node',
            name='ball_tracker_node',
            output='screen',
            parameters=['/homework1_ball_tracker/config/params.yaml'],
            remappings=[
                ('/rsd455_img', '/camera/image_raw'),
                ('/rsd455_depth', '/camera/depth/image_raw'),
                ('/ball_position', '/ball_position'),
                ('/target_pixel_coords', '/target_pixel_coords')
            ]
        ),
        Node(
            package='image_view',
            executable='image_view',
            name='image_view',
            output='screen',
            remappings=[
                ('/image', '/camera/image_raw')
            ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '/homework1_ball_tracker/rviz/ball_tracker.rviz']
        )
    ])