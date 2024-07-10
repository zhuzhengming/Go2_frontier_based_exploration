from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
 
def generate_launch_description():
    return LaunchDescription([
        # node1
        Node(
            package='control',
            executable='control',
            name='control',
            output='screen'
        ),
        
        # ros1_bridge
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros1_bridge', 'dynamic_bridge'],
            output='screen'
        )
    ])