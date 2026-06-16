
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

import yaml


def generate_launch_description():

    # Add cumotion planner node
    xrdf_path = os.path.join(
        get_package_share_directory('robot_control'),
        'urdf', 'arm5dof.xrdf'
    )
    urdf_path = os.path.join(
        get_package_share_directory('combined_robot_description'),
        'urdf', 'combined_robot.urdf.xacro'
    )

    moveit_config = (
        MoveItConfigsBuilder("combined_robot", package_name='combined_movit_config')
        .robot_description(file_path= urdf_path)
        .robot_description_semantic(file_path='config/combined_robot.srdf')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl', 'pilz_industrial_motion_planner'])
        .joint_limits(file_path='config/joint_limits.yaml')
		.to_moveit_configs()
		
    )

    # Add cuMotion to list of planning pipelines.
    cumotion_config_file_path = os.path.join(
        get_package_share_directory('isaac_ros_cumotion_moveit'),
        'config',
        'isaac_ros_cumotion_planning.yaml'
    )
    with open(cumotion_config_file_path) as cumotion_config_file:
        cumotion_config = yaml.safe_load(cumotion_config_file)
    moveit_config.planning_pipelines['planning_pipelines'].insert(0,'isaac_ros_cumotion')
    moveit_config.planning_pipelines['isaac_ros_cumotion'] = cumotion_config
    moveit_config.planning_pipelines['default_planning_pipeline'] = 'isaac_ros_cumotion'

    # The current Franka asset in Isaac Sim 2023.1.1 tends to drift slightly from commanded joint
    # positions, which prevents trajectory execution if the drift exceeds `allowed_start_tolerance`
    # for any joint; the default tolerance is 0.01 radians.  This is more likely to occur if the
    # robot hasn't fully settled when the trajectory is computed or if significant time has
    # elapsed between trajectory computation and execution. For this simulation use case,
    # there's little harm in disabling this check by setting `allowed_start_tolerance` to 0.
    #moveit_config.trajectory_execution['trajectory_execution']['allowed_start_tolerance'] = 0.05

    # Start the actual move_group node/action server
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
	    moveit_config.to_dict(),
	    {
		"trajectory_execution.allowed_execution_duration_scaling":2.0,		
		"default_planning_time":10,
	    },
	],
        arguments=['--ros-args', '--log-level', 'info'],
    )

    
    cumotion_planner_node = Node(
        name='cumotion_planner',
        package='isaac_ros_cumotion',
        namespace='',
        executable='cumotion_planner_node',
        parameters=[
            {
                'robot': xrdf_path,
                'urdf_path': urdf_path
            }
        ],
        output='screen',
    )

    # Static planning scene server
    static_planning_scene_server = Node(
        package='isaac_ros_cumotion',
        executable='static_planning_scene',
        name='static_planning_scene_server',
        output='screen',
        emulate_tty=True,
    )

    # RViz
    rviz_config_file = os.path.join(
        get_package_share_directory('arm_moveit_config'),
        'config','moveit.rviz',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # Static TF
    world2robot_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=['--frame-id', 'world', '--child-frame-id', 'base_link'],
    )

    # Publish TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[moveit_config.robot_description],
    )
    
    servo_node = Node(
            package='robot_control',
            executable='servo_node',
            name='servo_node',
            output='screen',
    )

    return LaunchDescription(
        [
            servo_node,
            #rviz_node,
            world2robot_tf_node,
            robot_state_publisher,
            move_group_node,
            #static_planning_scene_server,
            #cumotion_planner_node,
        ]
    )
