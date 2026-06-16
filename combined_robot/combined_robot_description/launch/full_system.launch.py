import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    pkg_combined = get_package_share_directory('combined_robot_description')

    robot_description = {
        'robot_description': ParameterValue(
            Command([
                'xacro ',
                os.path.join(pkg_combined, 'urdf', 'ackermann.urdf.xacro')
            ]),
            value_type=str
        )   
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    
    ackermann_chassis_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('combined_robot_description'),
                'launch',
                'ackermann_control.launch.py'
            )
        )
    )

    cmd_vel = TimerAction(
        period=15.0,  # delay in seconds to allow the controllers to configure
        actions=[
            Node(
                package='cmd_vel_ackermann_bridge',
                executable='cmd_vel_to_ackermann',
                name='cmd_vel_to_ackermann',
                output='screen',
            )
        ]
    )

    vslam_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('isaac_ros_visual_slam'),
                        'launch',
                        'isaac_ros_visual_slam_realsense.launch.py'
                    )
                )
            )
        ]
    )

    nvblox_launch = TimerAction(
        period=15.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('nvblox_examples_bringup'),
                        'launch',
                        'realsense_example.launch.py'
                    )
                ),
                launch_arguments={
                    'num_cameras': '1',
                    'run_realsense': 'True',
                    'attach_to_container': 'False',
                    'mode': 'static',
                }.items()
            )
        ]
    )

    nav2_launch = TimerAction(
        period=25.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('nav2_bringup'),
                        'launch',
                        'navigation_launch.py',
                    )
                ),
                launch_arguments={
                'params_file': '/workspaces/isaac_ros-dev/install/nvblox_examples_bringup/share/nvblox_examples_bringup/config/navigation/carter_nav2.yaml',
                'use_sim_time': 'False',
                'use_composition': 'False',
                }.items()
            )
        ]
    )


    robot_arm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('combined_robot_description'),
                'launch',
                 'cumotion_launch.py'
            )
        )
    ) 

    rviz_config_file = os.path.join(
        get_package_share_directory('combined_robot_description'),
        'rviz','combined_config.rviz',
    )
	

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    return LaunchDescription([
        #robot_state_publisher,
        ackermann_chassis_description,
        cmd_vel,
        vslam_launch,
        nvblox_launch,
        nav2_launch,
        robot_arm,
        rviz,
    ])
