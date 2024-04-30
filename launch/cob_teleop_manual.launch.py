import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro

def generate_launch_description():
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
             )
    
    tricycle_sim_path = os.path.join(
        get_package_share_directory('tricycle_sim')
    )

    urdf_file = os.path.join(
        tricycle_sim_path,
        'urdf',
        'cob4-25_0408.urdf'
    )

    doc = xacro.parse(open(urdf_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    node_robot_state_publisher = Node(
        package= 'robot_state_publisher',
        executable= 'robot_state_publisher',
        output= 'screen',
        parameters=[params]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'tricycle'],
        output= 'screen'
    )
    # spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
    #                     arguments=['-topic', 'robot_description',
    #                                '-entity', 'tricycle'],
    #                     output='screen')

    load_joint_state_broadcaster = ExecuteProcess(
        cmd = ['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output= 'screen'
    )

    load_tricycle_controller = ExecuteProcess(
        cmd = ['ros2', 'control', 'load_controller', '--set-state', 'active', 'tricycle_steering_controller'],
        output= 'screen'
    )

    twist_mux_params = os.path.join(get_package_share_directory('tricycle_sim'), 'config', 'twist_mux.yaml')
    twisit_mux = Node(
        package= 'twist_mux',
        executable= 'twist_mux',
        parameters= [twist_mux_params, {'use_sim_time': True}]
    )

    rviz2 = Node(
        package= 'rviz2',
        executable= 'rviz2',
        output= 'screen'
    )

    cmd_vel_bridge_Twist = Node(
        package= 'tricycle_sim',
        executable= 'cmd_vel_bridge_Twist_node.py',
        output= 'screen'
    )

    teleop_twist_joy_dir = get_package_share_directory('teleop_twist_joy')
    joy_config = LaunchConfiguration('joy_config', default= 'xbox')

    teleop_joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(teleop_twist_joy_dir, 'launch', 'teleop-launch.py')
        ),
        launch_arguments= {'joy_config': joy_config}.items()
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler= OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster]
            )
        ),
        RegisterEventHandler(
            event_handler= OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_tricycle_controller]
            )
        ),
        gazebo,        
        rviz2,
        node_robot_state_publisher,
        spawn_entity,
        twisit_mux,
        teleop_joy,
        cmd_vel_bridge_Twist
    ])