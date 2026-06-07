import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 데스크탑에서 실행: Nav2 (경로계획 + 장애물회피 + 제어).
    # 지도와 위치(map→odom)는 SLAM(slam.launch.py)이 제공하므로 여기선 navigation 만 띄운다.
    pkg_share = get_package_share_directory('autonomous_robot')
    params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    nav2_launch = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'params_file': params,
                'use_sim_time': 'false',
                'autostart': 'true',
            }.items(),
        ),
    ])
