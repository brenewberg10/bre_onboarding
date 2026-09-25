import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    launch_descr = []
    long_control_config = os.path.join(
        get_package_share_directory('long_control'),
        'config',
        'params.yaml'
    )

    kin_control_config = os.path.join(
        get_package_share_directory('kin_control'),
        'config',
        'params.yaml'
    )

    kin_control_node = Node(
        package='kin_control',
        executable='kin_control',
        name='KinControlNode',
        output='screen',
        parameters=[kin_control_config]
    )

    long_control_node = Node(
        package='long_control',
        executable='long_control',
        name='LongControlNode',
        output='screen',
        parameters=[long_control_config]
    )

    launch_descr.extend([
        kin_control_node,
        long_control_node
    ])

    return LaunchDescription(launch_descr)
                        

    
