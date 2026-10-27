import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess


def launch_setup(context):
    track = LaunchConfiguration('track').perform(context).lower()

    workspace = os.path.expanduser('~/bre_onboarding')
    map_directory = os.path.join(workspace, 'map')

    if track == 'monza':
        csv_path = os.path.join(map_directory, 'monza_mincurv.csv')
        datum_latitude = 45.61898065946136
        datum_longitude = 9.28112698795041
    elif track == 'lvms':
        csv_path = os.path.join(map_directory, 'lvms_center_min_curve.csv')
        datum_latitude = 36.27117541
        datum_longitude = -115.00764474
    else:
        raise RuntimeError(
            f"Unknown track '{track}'. Choose 'monza' or 'lvms'."
        )

    phase_4_config = os.path.join(
        get_package_share_directory('phase_4'),
        'config',
        'params.yaml'
    )

    kin_control_config = os.path.join(
        get_package_share_directory('kin_control'),
        'config',
        'params.yaml'
    )

    long_control_config = os.path.join(
        get_package_share_directory('long_control'),
        'config',
        'params.yaml'
    )

    # Both nodes must use the same ENU datum.
    localization_node = Node(
        package='phase_4',
        executable='localization_node',
        name='localization_node',
        output='screen',
        parameters=[
            phase_4_config,
            {
                'datum_latitude': datum_latitude,
                'datum_longitude': datum_longitude,
            },
        ],
    )

    planner_node = Node(
        package='phase_4',
        executable='planner_node',
        name='planner_node',
        output='screen',
        parameters=[
            phase_4_config,
            {
                'csv_path': csv_path,
                'datum_latitude': datum_latitude,
                'datum_longitude': datum_longitude,
            },
        ],
    )

    adapter_node = Node(
        package='phase_4',
        executable='vehicle_input_adapter',
        name='vehicle_input_adapter',
        output='screen',
        parameters=[phase_4_config],
    )

    kin_control_node = Node(
        package='kin_control',
        executable='kin_control',
        name='KinControlNode',
        output='screen',
        parameters=[kin_control_config],
    )

    long_control_node = Node(
        package='long_control',
        executable='long_control',
        name='LongControlNode',
        output='screen',
        parameters=[long_control_config],
    )

    simulator = ExecuteProcess(
        cmd=[
            os.path.expanduser(
                '~/PAIRsim/suPAIRsim_v1.3/suPAIRsim_v1.3.x86_64'
            )
        ],
        cwd=os.path.expanduser('~/PAIRsim/suPAIRsim_v1.3'),
        output='screen',
    )

    return [
        simulator,
        localization_node,
        planner_node,
        adapter_node,
        kin_control_node,
        long_control_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'track',
            default_value='monza',
            description="Track to load: 'monza' or 'lvms'",
        ),
        OpaqueFunction(function=launch_setup),
    ])
