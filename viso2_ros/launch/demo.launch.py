from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    arg = '/image'

    image_proc = Node(
      package='image_proc',
      namespace=arg,
      executable='image_proc',
      name='image_proc'
    )

    viso2_ros = Node(
      package='viso2_ros',
      namespace=arg,
      executable='mono_odometer',
      name='mono_odometer',
      parameters=[{
          'base_link_frame_id': arg,
          'camera_height': 1.00,
          'camera_pitch': 0.00,
      }],
      remappings=[
          ('image', arg + '/image_rect'),
      ]
    )

    return LaunchDescription([
        image_proc,
        viso2_ros,
    ])
