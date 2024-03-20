from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    camera_node = Node(
        package='image_transport_tutorials',
        executable='my_publisher'
    )

    media_pipe_node = Node(
        package='media_pipe',
        executable='media_pipe'
    )

    communication_node = Node(
        package='communication_w_pico',
        executable='communication_w_pico'
    )
    
    ld.add_action(camera_node)
    ld.add_action(media_pipe_node)
    ld.add_action(communication_node)
    return ld
