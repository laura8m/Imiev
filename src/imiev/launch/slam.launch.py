from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
import os


def generate_launch_description():
    imiev_dir = get_package_share_directory("imiev")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    slam_toolbox_dir = get_package_share_directory("slam_toolbox")
    
    # Params files
    nav2_params_file = os.path.join(imiev_dir, "config", "nav2_slam_params.yaml")
    rl_params_file = os.path.join(imiev_dir, "config", "dual_ekf_navsat_params.yaml")
    
    # 1. Gazebo + Robot Spawn (Reuse existing launch)
    gazebo_model_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(imiev_dir, "launch", "gazebo_model.launch.py")
        )
    )

    # 2. Local EKF ONLY (odom -> base_link) using same params but filtering nodes
    # We manually define the node here to avoid launching the global EKF from the other launch file
    ekf_local_node = launch_ros.actions.Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_odom",
        output="screen",
        parameters=[rl_params_file, {"use_sim_time": True}],
        remappings=[("odometry/filtered", "odometry/local")],
    )

    # 3. SLAM Toolbox (map -> odom)
    slam_params_file = os.path.join(imiev_dir, "config", "mapper_params_online_async.yaml")
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, "launch", "online_async_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "True",
            "slam_params_file": slam_params_file,
        }.items(),
    )

    # 4. Nav2 (Navigation only, no AMCL, no Map Server)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "True",
            "params_file": nav2_params_file,
            "autostart": "True",
        }.items(),
    )

    # 5. RViz
    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
    )

    return LaunchDescription(
        [
            gazebo_model_launch,
            ekf_local_node,
            slam_launch,
            nav2_launch,
            rviz_node,
        ]
    )
