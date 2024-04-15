
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameters_type import ParameterValue
import os
import xacro
import subprocess

def generate_launch_description():

    # xacro_file= os.path.join(get_package_share_directory("cob_hardware_config"),"robots","cob4-25","urdf","cob4-25.urdf-base.xacro")
    xacro_file= os.path.join(get_package_share_directory("cob_sim"),"urdf","cob4-25_0415_torso_arm.urdf")   # cob_right_arm_0415.urdf
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log"
    )

    # gazebo_node = IncludeLaunchDescription(
    #     launch_description_source=LaunchDescription([
    #         ExecuteProcess(
    #             cmd=["gazebo", "--verbose", "-s", "libgazebo_ros_factory.so"],
    #             output="screen",
    #             additional_env={'GAZEBO_MODEL_PATH': xacro_file},
    #         )
    #     ])
    # )

    nodes_to_start = [
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ]

    return LaunchDescription(nodes_to_start)
