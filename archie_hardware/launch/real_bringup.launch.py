"""
Launch para el robot REAL ARCHIE (sin Gazebo).

Lanza:
  1. robot_state_publisher  — publica TF a partir del URDF
  2. archie_hardware_node   — comunica con motores Dynamixel + expone la acción
                              arm_controller/follow_joint_trajectory

Para la capa de aplicación (MoveIt2 + write_word + tracer) lanza por separado:
  ros2 launch archie_master archie_bringup.launch.py use_sim:=false
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # --- Argumentos ---
    usb_port_arg = DeclareLaunchArgument(
        'usb_port',
        default_value='/dev/ttyUSB0',
        description='Puerto USB del U2D2 (ej. /dev/ttyUSB0)',
    )
    baud_rate_arg = DeclareLaunchArgument(
        'dxl_baud_rate',
        default_value='1000000',
        description='Baudrate de los motores Dynamixel',
    )
    # Perfil de movimiento Dynamixel (trapezoidal velocity profile)
    # profile_velocity    : 1 unit = 0.229 rev/min   |  50 → ~11.5 rpm
    # profile_acceleration: 1 unit ≈ 214.577 rev/min² |   8 → rampa suave
    # Aumentar para movimientos más rápidos; disminuir para más suaves.
    profile_vel_arg = DeclareLaunchArgument(
        'profile_velocity',
        default_value='30',
        description='Velocidad máxima Dynamixel (unidades: 0.229 rpm). 0 = sin límite.',
    )
    profile_acc_arg = DeclareLaunchArgument(
        'profile_acceleration',
        default_value='8',
        description='Aceleración Dynamixel (unidades: ~214.6 rpm²). 0 = sin límite.',
    )
    go_home_arg = DeclareLaunchArgument(
        'go_home_on_start',
        default_value='false',
        description='Si true, el robot se mueve suavemente a home (0 rad) al arrancar.',
    )

    # --- URDF (mismo que la simulación, sin gripper, con lápiz) ---
    # Se llama al ejecutable `xacro` en lugar de importar el módulo Python,
    # que es la forma recomendada en ROS2 launch files.
    xacro_file = PathJoinSubstitution(
        [FindPackageShare('archie_description'), 'urdf', 'manipulator_main.xacro']
    )
    robot_desc = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ', xacro_file,
            ' ee_marker:=true gripper:=false',
        ]),
        value_type=str,
    )

    # --- Nodos ---

    # robot_state_publisher: necesario para TF y para que MoveIt2 conozca el modelo
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_desc,
             'use_sim_time': False},
        ],
    )

    # Nodo de hardware: action server + joint_states publisher
    hardware_node = Node(
        package='archie_hardware',
        executable='archie_hardware_node',
        name='archie_hardware_node',
        output='screen',
        parameters=[
            {'use_sim_time':          False},
            {'usb_port':              LaunchConfiguration('usb_port')},
            {'dxl_baud_rate':         LaunchConfiguration('dxl_baud_rate')},
            {'profile_velocity':      LaunchConfiguration('profile_velocity')},
            {'profile_acceleration':  LaunchConfiguration('profile_acceleration')},
            {'go_home_on_start':      LaunchConfiguration('go_home_on_start')},
        ],
    )

    return LaunchDescription([
        usb_port_arg,
        baud_rate_arg,
        profile_vel_arg,
        profile_acc_arg,
        go_home_arg,
        rsp_node,
        hardware_node,
    ])
