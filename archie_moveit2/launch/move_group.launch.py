from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Argumento: true para Gazebo/sim, false para hardware real
    use_sim_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Usar tiempo de simulación (true=Gazebo, false=hardware real)',
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Cargar la configuración de MoveIt2 de ARCHIE
    moveit_config = MoveItConfigsBuilder("archie", package_name="archie_moveit2").to_moveit_configs()

    # use_sim_time se pasa dinámicamente desde el argumento de launch
    moveit_params = moveit_config.to_dict()

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_params, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        use_sim_arg,
        run_move_group_node,
    ])