import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # 是否使用模擬時間
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 取得 package 內的 xacro 檔案
    pkg_path = get_package_share_directory('wheeltec_robot_urdf')
    xacro_file = os.path.join(pkg_path, 'urdf', 'mini_mec_robot_gazebo.xacro')
    
    # 解析 Xacro 檔案
    robot_description_config = xacro.process_file(xacro_file)

    # 設定 robot_state_publisher 參數
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # 啟動 launch
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        node_robot_state_publisher,
        node_joint_state_publisher
    ])
