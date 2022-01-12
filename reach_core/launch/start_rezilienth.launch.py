from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    # Command,
    # FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "parameters_package",
            description="Package to look for study parameters yaml file.",
            default_value="tele_exam_system_moveit_config"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "parameters_filename",
            description="YAML file for study parameters.",
            default_value="reach_params.yaml"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz",
                              default_value="true",
                              description="Launch RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument("xacro_file",
                              default_value="reach_study_tele_exam_system.xacro",
                              description="Xacro file to parse.")
    )
    declared_arguments.append(
        DeclareLaunchArgument("description_package",
                              default_value="tele_exam_robot_description",
                              description="Xacro file to parse.")
    )
    declared_arguments.append(
        DeclareLaunchArgument("moveit_config_file",
                              default_value="reach_study_tele_exam_system.srdf.xacro",
                              description="Moveit config xacro file to parse.")
    )
    declared_arguments.append(
        DeclareLaunchArgument("moveit_config_package",
                              default_value="tele_exam_system_moveit_config",
                              description="Moveit configuration package.")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ros2_controllers_file",
            default_value="ros2_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "include_patient_chair",
            default_value="false",
            description="To include patient and chair in the robot description?",
        )
    )

    parameters_package = LaunchConfiguration("parameters_package")
    parameters_filename = LaunchConfiguration("parameters_filename")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    ros2_controllers_file = LaunchConfiguration("ros2_controllers_file")
    description_package = LaunchConfiguration("description_package")
    include_patient_chair = LaunchConfiguration("include_patient_chair")

    study_parameters = PathJoinSubstitution(
        [FindPackageShare(parameters_package), "config", parameters_filename]
    )
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", LaunchConfiguration("xacro_file")]
            ),
            " ",
            "include_patient_chair:=",
            include_patient_chair,
            " ",
        ]
    )
    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(moveit_config_package),"config", moveit_config_file]
            ),
        ]
    )

    ros2_controllers = PathJoinSubstitution(
        [FindPackageShare(parameters_package), "config", ros2_controllers_file]
    )

    kinematics_yaml = load_yaml("tele_exam_system_moveit_config", "config/kinematics.yaml")
    robot_description = {"robot_description": robot_description_content}
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    robot_reach_study_node = Node(
        package="reach_core",
        executable="robot_reach_study_node",
        name="robot_reach_study_node",
        output="screen",
        parameters=[
            study_parameters,
            robot_description,
            robot_description_semantic,
            robot_description_kinematics
        ],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    jtc_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
    )
    gantry_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gantry_joint_trajectory_controller", "--controller-manager", "/controller_manager"],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # Trajectory Execution Functionality
    moveit_simple_controllers_yaml = load_yaml(
        "tele_exam_system_moveit_config", "config/controllers.yaml"
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    # Planning Functionality
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(
        "tele_exam_system_moveit_config", "config/ompl_planning.yaml"
    )
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        # arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
            # robot_description_planning,
        ],
    )

    nodes_to_run = [robot_reach_study_node,
                    control_node,
                    robot_state_publisher_node,
                    joint_state_broadcaster_spawner,
                    run_move_group_node,
                    rviz_node,
                    # jtc_controller_spawner,
                    # gantry_controller_spawner,
                    ]

    return LaunchDescription(declared_arguments + nodes_to_run)