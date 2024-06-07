from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare the launch argument
    sleep_server_arg = DeclareLaunchArgument(
        'sleep_server',
        default_value='true',
        description='Whether to include the sleep_server node'
    )

    # Define the sample_bt_executor node
    sample_bt_executor_node = Node(
        package='btcpp_ros2_samples',
        executable='sample_bt_executor',
        output='screen',
        parameters=['/home/nhg-yc/bt_ws/src/BehaviorTree.ROS2/btcpp_ros2_samples/config/sample_bt_executor.yaml']
    )

    # Define the sleep_server node
    sleep_server_node = Node(
        package='btcpp_ros2_samples',
        executable='sleep_server',
        output='screen'
    )

    # Group the sleep_server node with a condition
    sleep_server_group = GroupAction(
        actions=[sleep_server_node],
        condition=IfCondition(LaunchConfiguration('sleep_server'))
    )

    # Create the launch description and add all actions
    ld = LaunchDescription()

    ld.add_action(sleep_server_arg)
    ld.add_action(sample_bt_executor_node)
    ld.add_action(sleep_server_group)

    return ld
