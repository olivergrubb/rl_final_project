import launch
import launch_ros.actions

def generate_launch_description():
    # Define the robot namespaces to use
    robot_namespaces = ["evader0", "chaser0"]

    # Create a list of nodes to launch
    nodes = []
    for robot_namespace in robot_namespaces:
        node = launch_ros.actions.Node(
            package='rl_methods',
            executable='action_publisher',
            name='action_publisher',
            namespace=robot_namespace,
            output='screen',
            parameters=[{'robot': robot_namespace}]
        )
        nodes.append(node)
        node = launch_ros.actions.Node(
            package='rl_methods',
            executable='colour_detection_node',
            name='colour_detection_node',
            namespace=robot_namespace,
            output='screen',
            parameters=[{'robot': robot_namespace}]
        )
        nodes.append(node)
        node = launch_ros.actions.Node(
            package='rl_methods',
            executable='lidar_processor_node',
            name='lidar_processor_node',
            namespace=robot_namespace,
            output='screen',
            parameters=[{'robot': robot_namespace}]
        )
        nodes.append(node)

    # Create a launch description with all the nodes
    ld = launch.LaunchDescription(nodes)

    return ld
