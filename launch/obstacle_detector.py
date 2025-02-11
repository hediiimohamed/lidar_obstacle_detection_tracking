import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the parameter file
    config_file = os.path.join(
        get_package_share_directory('lidar_detection_tracking'),
        'config',
        'obstacle_detector_params'
    )
    id = launch.LaunchDescription()  # Correct function name

    obstacle_detector = launch_ros.actions.Node(
        package='lidar_detection_tracking',
        executable='obstacle_detector',
        name='obstacle_detector',
        output='screen',
        parameters=[config_file]
        )
    id.add_action(obstacle_detector)  # Use add_action() (not add_actions) with a single object

    return id
