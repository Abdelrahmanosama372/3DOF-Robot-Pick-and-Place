import os
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    stmbot_pack_share_dir = get_package_share_directory(package_name="stmbot_description")
    model = os.path.join(stmbot_pack_share_dir, 'urdf', 'stmbot.urdf.xacro')

    robot_description = ParameterValue(Command(
        (['xacro ', model])
    ), value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=['-d',os.path.join(stmbot_pack_share_dir,'rviz','display.rviz')],
    )

    return LaunchDescription([
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz2_node
    ])
