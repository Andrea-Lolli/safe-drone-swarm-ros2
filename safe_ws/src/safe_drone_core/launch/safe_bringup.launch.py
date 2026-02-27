import os
from launch import LaunchDescription, LaunchContext
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# read parameter input from terminal
def launch_mission_managers(context: LaunchContext, *args, **kwargs):
    num_drones_str = LaunchConfiguration('num_drones').perform(context)
    num_drones = int(num_drones_str)
    
    nodes_to_start = []
    
    # generate N nodes based on drone id
    for i in range(1, num_drones + 1):
        drone_node = Node(
            package='safe_drone_core',
            executable='mission_manager',
            name=f'mission_manager_drone{i}',
            parameters=[{'drone_id': i}],
            output='screen'
        )
        nodes_to_start.append(drone_node)
        
    return nodes_to_start

def generate_launch_description():
    # parameters declaration
    num_drones_arg = DeclareLaunchArgument(
        'num_drones',
        default_value='1',
        description='Numero di cervelli mission_manager da avviare'
    )

    # start gazebo
    sdf_file = os.path.expanduser('~/safe_ws/simulation/safe_world.sdf')
    start_gazebo = ExecuteProcess(
        cmd=['ros2', 'launch', 'ros_gz_sim', 'gz_sim.launch.py', f'gz_args:=-r {sdf_file}'],
        output='screen'
    )

    # start bridge (ready for 10 drones)
    bridge_args = ['/visualization_marker@visualization_msgs/msg/Marker@gz.msgs.Marker']
    bridge_remappings = []
    
    for i in range(1, 11):
        bridge_args.append(f'/drone{i}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist')
        bridge_args.append(f'/model/drone{i}/pose@geometry_msgs/msg/Pose@gz.msgs.Pose')
        bridge_remappings.append((f'/model/drone{i}/pose', f'/drone{i}/ground_truth/pose'))

    start_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridge_args,
        remappings=bridge_remappings,
        output='screen'
    )

    # start sensors
    sensor_1 = Node(
        package='safe_drone_core',
        executable='mock_sensor',
        name='mock_sensor_1', 
        parameters=[{
            'sensor_id': 'SAFE_EDV_001',
            'x': 24.3, 'y': 23.2, 'z': 0.0, 'tx_power': -20.0
        }],
        output='screen'
    )
    
    sensor_2 = Node(
        package='safe_drone_core',
        executable='mock_sensor',
        name='mock_sensor_2',
        parameters=[{
            'sensor_id': 'SAFE_EDV_002',
            'x': 21.9, 'y': 37.4, 'z': 4.5, 'tx_power': -20.0
        }],
        output='screen'
    )

    sensor_3 = Node(
        package='safe_drone_core',
        executable='mock_sensor',
        name='mock_sensor_3',
        parameters=[{
            'sensor_id': 'SAFE_EDV_003',
            'x': 5.4, 'y': 40.0, 'z': -1.5, 'tx_power': -20.0
        }],
        output='screen'
    )

    # start rviz
    start_rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    return LaunchDescription([
        num_drones_arg,
        start_gazebo,
        start_bridge,
        sensor_1,     
        sensor_2,
        sensor_3,
        start_rviz,
        OpaqueFunction(function=launch_mission_managers) 
    ])
