from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    node_ledstrip_sub = Node(
        package="robu",
        executable="ledsub",
        output="screen",
        # remappings=
        # parameters=
        # arguments=
    )

    exec_ledstrip_pub = ExecuteProcess(

        cmd=['terminator','-e', '\'bash -c "source /opt/ros/humble/setup.bash; source /home/robu/work/robu_bhme21_ws/install/setup.bash; ros2 run robu ledpub; exec bash"\''],
        shell=True,

    )


    # node_ledstrip_pub = Node(
    #     package="robu",
    #     executable="plf01_pub",
    #     output="screen",
    #     # remappings=
    #     # parameters=
    #     # arguments=
    # )


    ld = LaunchDescription()
    ld.add_action(node_ledstrip_sub)
    ld.add_action(exec_ledstrip_pub)
    #ld.add_action(node_ledstrip_pub)
    
    return ld