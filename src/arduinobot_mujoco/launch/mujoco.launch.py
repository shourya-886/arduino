"""Launch the Arduinobot model in the native MuJoCo viewer."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_prefix


def generate_launch_description():
    default_model = [
        FindPackageShare("arduinobot_mujoco"),
        "/urdf/my_robot_mujoco.xml",
    ]
    runner = PathJoinSubstitution([
        get_package_prefix("arduinobot_mujoco"),
        "lib",
        "arduinobot_mujoco",
        "mujoco_sim.py",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "model",
            default_value=default_model,
            description="Absolute path to the MuJoCo XML model to simulate.",
        ),
        ExecuteProcess(
            cmd=[
                "python3",
                runner,
                LaunchConfiguration("model"),
            ],
            output="screen",
        ),
    ])
