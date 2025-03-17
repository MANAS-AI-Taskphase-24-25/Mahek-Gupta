import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='group_chat',
            executable='chat_node_1',
            name='chat_node_1',
            output='screen',
            prefix='gnome-terminal --'
        ),
        launch_ros.actions.Node(
            package='group_chat',
            executable='chat_node_2',
            name='chat_node_2',
            output='screen',
            prefix='gnome-terminal --'
        )
    ])

