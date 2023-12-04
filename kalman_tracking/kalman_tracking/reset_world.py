from std_srvs.srv import Empty
from gazebo_msgs.srv import SpawnEntity
from gazebo_msgs.srv import DeleteEntity
import os
import time
import rclpy
import random
from evader_navigation_node import EvaderNavigationNode
import sys

def reset_world(node, mode):
    node.get_logger().warning(f'RESETTING.')

    # Reset world in gazebo then spawn a goal in random location in map
    gazebo_service_call(Empty.Request(), Empty, '/reset_simulation', False)

    # Delete previous goal
    request = DeleteEntity.Request()
    request.name = 'goal'
    gazebo_service_call(request, DeleteEntity, '/delete_entity', False)

    # Get random goal pose
    goal_pose = get_goal_pose()

    # Spawn goal
    sdf_file_path = os.path.join("goal.sdf")
    request = SpawnEntity.Request()
    request.name = 'goal'
    request.xml = open(sdf_file_path, 'r').read()
    request.initial_pose.position.x = goal_pose[0]
    request.initial_pose.position.y = goal_pose[1]
    request.initial_pose.position.z = 0.0
    # stabalise to stop async spawning issues
    time.sleep(0.5)
    gazebo_service_call(request, SpawnEntity, '/spawn_entity', False)
    
    if mode == "auto":
        evader_navigation_node = EvaderNavigationNode(goal_pose)

        # while the evader_navigation_node is alive
        while rclpy.ok():
            rclpy.spin(evader_navigation_node)
            if not evader_navigation_node.is_alive():
                break

def gazebo_service_call(request, msg_type ,service_name, return_result=False):
        node = rclpy.create_node('reset_simulation_node')
        client = node.create_client(msg_type, service_name)
        if not client.service_is_ready():
            client.wait_for_service()
            node.get_logger().warning(f'{service_name} service is not ready.')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        
        node.destroy_node()
        
        if return_result:
            return future.result()

def get_goal_pose():
    x = random.uniform(-6.0, 6.0)
    y = 6.0

    return (x, y)
        
if __name__ == '__main__':
    rclpy.init()
    mode = sys.argv[1]
    node = rclpy.create_node('reset_simulation_node')
    reset_world(node, mode)
    node.destroy_node()
    rclpy.shutdown()