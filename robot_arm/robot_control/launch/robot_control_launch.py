from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_control')
    rviz_config = os.path.join(pkg_share, 'urdf', 'arm5dof.rviz')

    urdf_file = os.path.join(pkg_share, 'urdf', 'arm5dof.urdf')
    with open(urdf_file, 'r') as infp:  
        robot_description = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='rviz2',         
            executable='rviz2',      
            name='rviz2',            
            output='screen',
            arguments=['-d', rviz_config],  # Will fail if the .rviz file doesn't exist
        ),
        Node(
            package='robot_control',
            executable='hardware_node',
            name='hardware_node',
            output='screen',
        ),
        
    ])
