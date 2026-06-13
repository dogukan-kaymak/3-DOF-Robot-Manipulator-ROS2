from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Load MoveIt configuration
    moveit_config = MoveItConfigsBuilder("mechanism_308", package_name="mechanism_308_moveit_config").to_moveit_configs()

    # Robot State Publisher (Publishes TF from URDF)
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        respawn=True,
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # MoveIt MoveGroup Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": False},
        ],
    )

    # RViz2 Node
    rviz_config = os.path.join(get_package_share_directory('mechanism_308_moveit_config'), 'config', 'moveit.rviz')
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # ROS 2 Control Node (Hardware Interface Manager)
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            os.path.join(get_package_share_directory('mechanism_308_moveit_config'), 'config', 'ros2_controllers.yaml'),
        ],
        output="screen",
    )

    # Spawner for Joint State Broadcaster (Reads real position from hardware and publishes to /joint_states)
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    # Spawner for Arm Trajectory Controller (Sends trajectories to hardware)
    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mech_arm_controller"],
    )

    return LaunchDescription([
        rsp_node,
        move_group_node,
        rviz_node,
        ros2_control_node,
        jsb_spawner,
        arm_spawner
    ])
