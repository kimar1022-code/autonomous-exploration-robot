import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('autonomous_robot')
    urdf_path = os.path.join(pkg_share, 'urdf', 'robot.urdf')

    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    base_controller_node = Node(
        package='autonomous_robot',
        executable='base_controller'
    )

    camera_node = Node(
        package='autonomous_robot',
        executable='camera_node'
    )

    return LaunchDescription([
        robot_state_pub_node,
        base_controller_node,
        camera_node,
    ])
