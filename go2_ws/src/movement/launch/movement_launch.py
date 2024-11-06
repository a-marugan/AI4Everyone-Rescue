import launch

from launch_ros.actions import Node
    
def generate_launch_description():

    return launch.LaunchDescription([
        Node(
            package="movement",
            executable="move_to_location",
            name="move_to_location",
            output='screen'
        ),
        Node(
            package="movement",
            executable="path_planning",
            name="path_planning",
            output='screen'
        )
    ])