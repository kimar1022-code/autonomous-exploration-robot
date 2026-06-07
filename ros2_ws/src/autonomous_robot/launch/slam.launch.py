import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 데스크탑(연산 기기)에서 실행: slam_toolbox online_async 모드로 /scan + /odom 받아 /map 생성
    pkg_share = get_package_share_directory('autonomous_robot')
    params_file = os.path.join(pkg_share, 'config', 'slam_toolbox.yaml')

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        # use_sim_time=False: 실제 로봇(Pi)의 실시간 시계 사용
        parameters=[params_file, {'use_sim_time': False}],
    )

    return LaunchDescription([slam_node])
