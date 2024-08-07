from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
 
def generate_launch_description():

    declare_method_arg = DeclareLaunchArgument(
        'method', default_value='standard',
        description='Method to determine the topic name')

    method = LaunchConfiguration('method')

    return LaunchDescription([
        declare_method_arg,
        # node1
        Node(
            package='control',
            executable='control',
            name='control',
            parameters=[
                {'method': method}
                ],
            output='screen'
        ),
        
        # ros1_bridge
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros1_bridge', 'dynamic_bridge'],
            output='screen'
        )
    ])