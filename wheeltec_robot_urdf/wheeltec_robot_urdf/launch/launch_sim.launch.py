import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    package_name = 'wheeltec_robot_urdf'

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'display.launch.py')]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 確保不會重複開 Gazebo
    gazebo = ExecuteProcess(
        cmd=['pkill -9 gzserver; pkill -9 gzclient; sleep 2; ros2 launch gazebo_ros gazebo.launch.py'],
        shell=True,
        output='screen'
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'wheeltec_robot'],
        output='screen',
        remappings=[
            ('/cmd_vel', '/mecanum_drive_controller/commands')
        ]
    )

    load_controllers = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["mecanum_drive_controller", "--controller-manager", "/controller_manager"],
            output="screen"
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen"
        )
    ]

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        *load_controllers
    ])
