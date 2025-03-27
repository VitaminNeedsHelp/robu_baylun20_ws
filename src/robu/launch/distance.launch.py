from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    node_distance_sub = Node(
        package='robu',
        executable='obstacle_avoidance',
        name='node_name',
        output='screen',
        parameters=[
            {'parameter_name': 'parameter_value'}
        ]
    )

    exec_distance_pub = ExecuteProcess(
        cmd=['terminator','-e', '\'bash -c "source /opt/ros/humble/setup.bash; source /home/robu/work/robu_bhme21_ws/install/setup.bash; ros2 run robu distance_sensor; exec bash"\''],
        shell=True,
        )

    ld = LaunchDescription()
    ld.add_action(node_distance_sub)
    ld.add_action(exec_distance_pub)
    return ld