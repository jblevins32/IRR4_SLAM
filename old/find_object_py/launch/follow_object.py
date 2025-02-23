import launch
import launch_ros.actions
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return launch.LaunchDescription([
        # Include the TurtleBot3 camera stack launch file
        # launch.actions.IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         PathJoinSubstitution([
        #             FindPackageShare('turtlebot3_bringup'), 'launch', 'camera_robot.launch.py'
        #         ])
        #     ])
        # ),

        # Launch the CV node
        launch_ros.actions.Node(
            package='find_object_py',
            executable='subscriber',
            name='object_tracker',
            output='screen'
        ),

        # Launch the control node
        launch_ros.actions.Node(
            package='rotate_robot',
            executable='rotate_robot_pubsub',
            name='robot_controller',
            output='screen'
        )
    ])
