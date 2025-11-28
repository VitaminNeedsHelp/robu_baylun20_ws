from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    arg_threshold = DeclareLaunchArgument(
        'threshold', default_value='28.0',
        description='Dieser Wert legt die Temperaturschwelle fest!\
            Wenn die gemessene Temperatur diesen Wert überschreitet, wird eine Warnung ausgegeben.')
    
    arg_hz = DeclareLaunchArgument(
        'hz', default_value='1.0',
        description='Legt die Messgeschwindigkeit in Hz fest.')

    arg_mean = DeclareLaunchArgument(
        'mean', default_value='25.0',
        description='Legt den Mittelwert der Temperatur fest.')
    
    arg_amp = DeclareLaunchArgument(
        'amp', default_value='5.0',
        description='Maximale Schwankung um den Mittelwert der Temperatur.')
    
    node_temp_sensor = Node(
        package='temp_monitor_demo',
        executable='temp_sensor',
        name='temp_sensor',
        parameters=[{
            'hz': LaunchConfiguration('hz'),
            'mean': LaunchConfiguration('mean'),
            'amp': LaunchConfiguration('amp')
        }]
    )

    node_temp_monitor = Node(
        package='temp_monitor_demo',
        executable='temp_monitor',
        name='temp_monitor',
        parameters=[{
            'threshold': LaunchConfiguration('threshold')}]
    )



    ld = LaunchDescription()
    ld.add_action(arg_threshold)
    ld.add_action(arg_hz)
    ld.add_action(arg_mean)
    ld.add_action(arg_amp)
    ld.add_action(node_temp_sensor)
    ld.add_action(node_temp_monitor)
    return ld