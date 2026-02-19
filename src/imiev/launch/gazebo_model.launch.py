import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Nombre del robot y paquete
    robot_name = 'imiev'
    package_name = 'imiev'
    
    # Obtenemos la ruta de instalación del paquete (.../install/imiev/share/imiev)
    pkg_share = get_package_share_directory(package_name)
    
    # Obtenemos la carpeta 'share' general (.../install/imiev/share)
    # Esto permite a Gazebo resolver "package://imiev" correctamente
    install_share_path = os.path.dirname(pkg_share)

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        install_share_path
    )

    # --------------------------
    # Ruta del archivo XACRO
    # --------------------------
    xacro_file = os.path.join(
        get_package_share_directory(package_name),
        'models',
        'imiev',
        'robot.xacro'
    )

    # Convertir XACRO a URDF
    robot_description = xacro.process_file(xacro_file).toxml()
    
    # --------------------------
    # Ruta del archivo world
    # --------------------------
    world_file = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'sonoma_raceway.world'
    )
    
    # --------------------------
    # Launch de Gazebo (ros_gz_sim)
    # --------------------------
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': f'-r -v 4 "{world_file}"',  # -r reinicia, -v 4 nivel de logging
            'on_exit_shutdown': 'true'
        }.items()
    )
    
    # --------------------------
    # Node para spawn del robot
    # --------------------------
    spawn_robot_node = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', robot_name,
                '-topic', 'robot_description',
                '-x', '100.0',  # Posición X (sobre la pista)
                '-y', '100.0',  # Posición Y (sobre la pista)
                '-z', '7.0',    # Posición Z (Altura)
                '-R', '0.0',    # Roll
                '-P', '0.0',    # Pitch
                '-Y', '0.0'     # Yaw (Rotación)
            ],
            output='screen'
        )
    
    # --------------------------
    # Node de robot_state_publisher
    # --------------------------
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )
    
    # --------------------------
    # Node para el bridge ROS <-> Gazebo
    # --------------------------
    bridge_params = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'imiev_bridge.yaml'
    )
    
    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p', f'config_file:={bridge_params}'
        ],
        output='screen'
    )
    
    # --------------------------
    # Construir LaunchDescription
    # --------------------------
    ld = LaunchDescription()
    ld.add_action(set_env_vars_resources)
    ld.add_action(gazebo_launch)
    ld.add_action(spawn_robot_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(ros_gz_bridge_node)
    
    return ld
