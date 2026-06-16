from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import TimerAction

def generate_launch_description():
    pkg_share = FindPackageShare("ackermann_chassis_description")

    # ---- Launch args (match your xacro) ----
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    serial_port = LaunchConfiguration("serial_port")
    serial_baud = LaunchConfiguration("serial_baud")

    declared_arguments = [
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="If true, uses a fake ros2_control system instead of the Arduino hardware.",
        ),
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyACM0",
            description="Serial device for the Arduino (only used when use_fake_hardware:=false).",
        ),
        DeclareLaunchArgument(
            "serial_baud",
            default_value="115200",
            description="Baud rate for the Arduino serial link (only used when use_fake_hardware:=false).",
        ),
    ]

    # ---- Paths ----
    xacro_file = PathJoinSubstitution([pkg_share, "urdf", "ackermann.urdf.xacro"])
    controllers_yaml = PathJoinSubstitution([pkg_share, "config", "ackermann_controllers.yaml"])
    # RViz config is optional; if you don’t have one, RViz will still open fine.
    rviz_config = PathJoinSubstitution([pkg_share, "rviz", "view_model.rviz"])

    # ---- Xacro → URDF string (pass launch args into xacro) ----
    robot_description_content = Command([
        "xacro ",
        xacro_file,
        " use_fake_hardware:=", use_fake_hardware,
        " serial_port:=", serial_port,
        " serial_baud:=", serial_baud,
    ])
    robot_description = ParameterValue(robot_description_content, value_type=str)

    # ---- Nodes ----
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    # Controller Manager (ros2_control_node)
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description}, controllers_yaml],
        output="both",
    )

    # Spawners (delay a bit to ensure CM is up)
    jsb_spawner = TimerAction(
        period=1.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
        )]
    )

    steering_spawner = TimerAction(
        period=1.5,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["steering_controller", "--controller-manager", "/controller_manager"],
            output="screen",
        )]
    )

    rear_vel_spawner = TimerAction(
        period=2.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["rear_velocity_controller", "--controller-manager", "/controller_manager"],
            output="screen",
        )]
    )
    
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    return LaunchDescription(
        declared_arguments + [
            #rsp,
            controller_manager,
            jsb_spawner,
            steering_spawner,
            rear_vel_spawner,
            #rviz,
        ]
    )
