import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='move_base_node,
            executable='move_base_node',
            output='screen',
            name='move_base_node')
    ])