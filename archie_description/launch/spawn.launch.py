import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
import math
def generate_launch_description():

    # --- CONFIGURACIÓN DE HERRAMIENTA ---
    # Cambia estos valores a 'true' o 'false' según lo que quieras simular
    usar_garra = 'false'
    usar_lapiz = 'true'
    # ------------------------------------

    pkg_name = 'archie_description'
    pkg_share = get_package_share_directory(pkg_name)
    xacro_file = os.path.join(pkg_share, 'urdf', 'manipulator_main.xacro')

    # Procesar Xacro pasándole nuestras variables
    robot_description_config = xacro.process_file(
        xacro_file, 
        mappings={'ee_marker': usar_lapiz, 'gripper': usar_garra}
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_share, '..')
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': 'empty.sdf -r'}.items() 
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'archie', '-Y', str(0.0)],
        output='screen'
    )

    load_joint_state_broadcaster = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    load_arm_controller = Node(
        package="controller_manager", executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # Lista base de nodos a lanzar
    spawn_nodes = [
        set_env_vars_resources,
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        load_joint_state_broadcaster,
        load_arm_controller,
        clock_bridge
    ]

    # Solo agregamos el controlador de la garra si la garra está activada
    if usar_garra == 'true':
        load_gripper_controller = Node(
            package="controller_manager", executable="spawner",
            arguments=["ee_group_controller", "--controller-manager", "/controller_manager"],
        )
        spawn_nodes.append(load_gripper_controller)

    return LaunchDescription(spawn_nodes)