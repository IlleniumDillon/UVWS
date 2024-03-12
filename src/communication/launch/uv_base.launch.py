from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ti = Node(package="communication",executable="communication_node",name="t1",parameters=[
        {
            "serial_port": "/dev/ttyACM0"
        },{
            "name": "uv01"
        }
        ]
    )
    return LaunchDescription([ti])
        