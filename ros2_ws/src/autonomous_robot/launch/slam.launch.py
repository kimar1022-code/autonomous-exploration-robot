import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 데스크탑(연산 기기)에서 실행: SLAM(지도 그리기) + 라이다 프레임 다리
    pkg_share = get_package_share_directory('autonomous_robot')
    params_file = os.path.join(pkg_share, 'config', 'slam_toolbox.yaml')

    # ⚠️ slam_toolbox 는 Jazzy 에서 LifecycleNode 라 일반 Node 로 띄우면 멈춘다.
    #    공식 online_async_launch.py 를 include 하면 configure→activate 전환을 자동 처리한다.
    slam_launch = os.path.join(
        get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')

    # 라이다(/scan)는 frame_id="laser" 로 발행되는데 로봇 URDF TF엔 lidar_link 만 있다.
    # 둘은 같은 위치이므로 0 오프셋으로 이어 laser 를 TF 트리에 연결한다(SLAM이 scan 을 쓰려면 필요).
    lidar_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_to_laser_bridge',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--frame-id', 'lidar_link', '--child-frame-id', 'laser'],
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch),
        launch_arguments={
            'slam_params_file': params_file,
            'use_sim_time': 'false',
        }.items(),
    )

    # IMU 융합: 바퀴 odom(/odom) + IMU 자이로(/imu/data_raw) → 정확한 odom→base_link TF.
    # base_controller 는 publish_tf=False 로 TF 를 양보해야 충돌하지 않는다.
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'ekf.yaml'),
                    {'use_sim_time': False}],
    )

    return LaunchDescription([lidar_bridge, ekf, slam])
