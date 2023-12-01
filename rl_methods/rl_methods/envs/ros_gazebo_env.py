
import gymnasium as gym
from gymnasium import spaces
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from std_msgs.msg import Float32MultiArray
from gazebo_msgs.srv import SpawnEntity
from gazebo_msgs.srv import DeleteEntity
import os
import time
import math

# Currently set up for one chaser and one evader, reward is for evader

class RosGazebo(gym.Env):
    def __init__(self):
        # Define action and observation space
        # First number defines linear velocity, second number defines angular velocity
        self.action_space = spaces.Box(low=np.array([0, -0.5]), high=np.array([1, 0.5]), shape=(2,), dtype=np.float32)

        self.observation_space = spaces.Dict({
            # Condensed lidar data from 360 degrees
            "lidar": spaces.Box(low=0, high=3.5, shape=(72,), dtype=np.float32),
            # Three binary numbers representing whether the robot sees red, green, and blue in that order
            # Changed to 2 to exclude reverse and stop
            "camera": spaces.MultiBinary(3),
            # Distance to goal
            "distance_to_goal": spaces.Box(low=0, high=15, shape=(1,), dtype=np.float32),
            # Distance to chaser
            "distance_to_chaser": spaces.Box(low=0, high=15, shape=(1,), dtype=np.float32),
            # Robot odom
            "robot_odom": spaces.Box(low=-15, high=15, shape=(3,), dtype=np.float32),
            # Angle to goal
            "angle_to_goal": spaces.Box(low=-math.pi, high=math.pi, shape=(1,), dtype=np.float32),

        })
        rclpy.init(args=None)

        # Create publisher for trainer
        self.node = Node("Trainer")
        self.trainer_publisher = self.node.create_publisher(Float32MultiArray, '/evader0/trainer', 10)

        self.last_recieved_message = None
        self.goal_pose = None
        self.counter = 0

    # Tested
    def reset(self, seed=None, options=None):
        self.node.get_logger().warning(f'RESETTING.')
        super().reset(seed=seed, options=options)
        # Reset world in gazebo then spawn a goal in random location in map
        self._gazebo_service_call(Empty.Request(), Empty, '/reset_simulation', False)
        
        # Delete previous goal
        request = DeleteEntity.Request()
        request.name = 'goal'
        self._gazebo_service_call(request, DeleteEntity, '/delete_entity', False)
       
        self.goal_pose = self._goal_pose()

        sdf_file_path = os.path.join("goal.sdf")

        request = SpawnEntity.Request()
        request.name = 'goal'
        request.xml = open(sdf_file_path, 'r').read()
        request.initial_pose.position.x = self.goal_pose[0]
        request.initial_pose.position.y = self.goal_pose[1]
        request.initial_pose.position.z = 0.0
        # stabalise to stop async spawning issues
        time.sleep(1)
        self._gazebo_service_call(request, SpawnEntity, '/spawn_entity', False) 
        
        self.last_recieved_message = None

        observation = self._get_obvs()
        with open('rewards.txt', 'a') as f:
            f.write(f'{"EPISODE END"}\n')
        
        self.counter = 0

        return observation, {}

    def step(self, action):
        self.counter += 1
        # Publish direction for discrete_action_publisher
        msg = Float32MultiArray()
        msg.data = [float(i) for i in action]
        self.trainer_publisher.publish(msg)

        # Get observation from lidar and camera
        observation = self._get_obvs()

        truncated = False
        if self.counter % 50 == 0:
            self.node.get_logger().info(f'Truncated')
            truncated = True

        # Get reward from reward function
        reward = self._get_reward(observation, action, truncated)

        # Check if episode is done
        terminated = False
        if self._is_done(observation) or self._is_caught(observation) or self._is_collided(observation):
            terminated = True

        # Record reward in text file
        with open('rewards.txt', 'a') as f:
            f.write(f'{reward}\n')

        return observation, reward, terminated, truncated, {}


    def render(self, mode='human'):
        # Render the environment
        pass

    def _get_reward(self, observation, action, truncated):
               
        # Small scaling negative reward for distance away from goal inverse square law
        distance_to_goal = observation['distance_to_goal']
        distance_scaling_factor = 5  # Experiment with different values        

        reward = (1 / (int(math.ceil(distance_to_goal))**2)) * distance_scaling_factor
        
        # Small negative reward for not facing the goal scales with distance to goal
        if abs(observation['angle_to_goal']) > 0.5:
            reward -= 0.3

        # Punish for not moving forward
        if action[0] < 0.3:
            reward -= 0.1

        # small negative reward each step to encourage faster completion
        reward -= 0.01

        # Normalize rewards to [-1, 1]
        normalization_factor = 0.1  # Experiment with different values
        reward *= normalization_factor
        
        # Very Large reward for evader if goal is reached
        if self._is_done(observation):
            reward = 100 * normalization_factor  # Adjust scaling for large rewards
        elif truncated:
            reward = -100 * normalization_factor
        # Very Large negative reward if chaser catches evader
        elif self._is_caught(observation):
            reward = -100 * normalization_factor  # Adjust scaling for large rewards
        
        # Large negative reward if evader collides with wall
        elif self._is_collided(observation):
            reward = -100 * normalization_factor  # Adjust scaling for large rewards
        
        return reward


    def _is_done(self, observation):
        # Check if goal is reached
        # Goal reached when pose of evader is within a certain distance of goal
        self._get_last_message('/evader0/odom', Odometry)
        evader_odom = self.last_recieved_message
        evader_pose = [evader_odom.pose.pose.position.x, evader_odom.pose.pose.position.y]

        # Find distance between chaser and evader, is distance is less than threshold then evader is caught
        distance = np.linalg.norm(np.array(self.goal_pose) - np.array(evader_pose))

        if distance < 1:
            self.node.get_logger().info(f'Reached Goal!')
            return True
        else:
            return False

    def _is_caught(self, observation):
        # Check if evader is caught
        # Evader is caught when chaser is within a certain distance
        self._get_last_message('/chaser0/odom', Odometry)
        chaser_odom = self.last_recieved_message
        chaser_pose = [chaser_odom.pose.pose.position.x, chaser_odom.pose.pose.position.y]
        self.chaser_pose = chaser_pose

        self._get_last_message('/evader0/odom', Odometry)
        evader_odom = self.last_recieved_message
        self.evader_pose = [evader_odom.pose.pose.position.x, evader_odom.pose.pose.position.y]

        # Find distance between chaser and evader, is distance is less than threshold then evader is caught
        distance = np.linalg.norm(np.array(self.chaser_pose) - np.array(self.evader_pose))
        if distance < 0.5:
            self.node.get_logger().info(f'Caught!')
            return True
        else:
            return False

    def _is_collided(self, observation):
        # Check if evader has collided with wall
        # Evader has collided with wall when lidar reading is less than a certain distance

        self._get_last_message('/evader0/cleaned_lidar', Float32MultiArray)
        lidar = self.last_recieved_message
        counter = 0
        if lidar is not None:
            for distance in lidar:
                if distance < 0.2:
                    counter += 1
                    if counter >=4:
                        self.node.get_logger().info(f'Collided!')
                        return True
            return False
        else:
            return False

    def _get_obvs(self):
        # Get observation from lidar and camera
        self._get_last_message('/evader0/cleaned_lidar', Float32MultiArray)
        lidar = np.array(self.last_recieved_message)
        self._get_last_message('/evader0/visible_color', String)
        camera = self.last_recieved_message
        # Loop through the string and replace F with 0 and T with 1
        camera = np.array([int(x) for x in camera.replace('F', '0').replace('T', '1')])
        self._get_last_message('/evader0/odom', Odometry)
        evader_odom = self.last_recieved_message
        self.evader_pose = [evader_odom.pose.pose.position.x, evader_odom.pose.pose.position.y]
        distance_to_goal = np.array([np.linalg.norm(np.array(self.goal_pose) - np.array(self.evader_pose))])
       
        # Calculate angle from robot facing direction to goal
        robot_heading = math.atan2(
            2.0 * (evader_odom.pose.pose.orientation.w * evader_odom.pose.pose.orientation.z +
                evader_odom.pose.pose.orientation.x * evader_odom.pose.pose.orientation.y),
            1.0 - 2.0 * (evader_odom.pose.pose.orientation.y**2 + evader_odom.pose.pose.orientation.z**2)
        )
        goal_heading = math.atan2(self.goal_pose[1] - self.evader_pose[1], self.goal_pose[0] - self.evader_pose[0])
        angle = goal_heading - robot_heading

        self._get_last_message('/chaser0/odom', Odometry)
        chaser_odom = self.last_recieved_message
        self.chaser_pose = [chaser_odom.pose.pose.position.x, chaser_odom.pose.pose.position.y]
        distance_to_chaser = np.array([np.linalg.norm(np.array(self.chaser_pose) - np.array(self.evader_pose))])
        
        observation = {
            "lidar": lidar.astype(np.float32),
            "camera": camera,
            "distance_to_goal": distance_to_goal.astype(np.float32),
            "distance_to_chaser": distance_to_chaser.astype(np.float32),
            "robot_odom": np.array([self.evader_pose[0], self.evader_pose[1], evader_odom.pose.pose.orientation.z]).astype(np.float32),
            "angle_to_goal": np.array([angle]).astype(np.float32),
        }
        return observation
    
    def _get_last_message(self, topic, data_type):
        self.subscription = self.node.create_subscription(data_type, topic, self._singular_msg, 10)
        rclpy.spin_once(self.node)
        timeout_sec = 5.0
        start_time = self.node.get_clock().now()

        while not self.last_recieved_message:
            rclpy.spin_once(self.node, timeout_sec=1.0)

            if (self.node.get_clock().now().to_msg().sec - start_time.to_msg().sec) > timeout_sec:
                self.node.get_logger().warning(f'Timeout reached. No message received on topic {topic}.')
                break

        self.node.destroy_subscription(self.subscription)
            
    def _singular_msg(self, msg):
        if hasattr(msg, 'data'):
            self.last_recieved_message = msg.data
        else:
            self.last_recieved_message = msg
       
    def _gazebo_service_call(self, request, msg_type ,service_name, return_result=False):
        node = rclpy.create_node('reset_simulation_node')
        client = node.create_client(msg_type, service_name)
        if not client.service_is_ready():
            client.wait_for_service()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        
        node.destroy_node()
        
        if return_result:
            return future.result()
        
    # returns tuple of x and y coordinates of goal avoiding robot start positions
    def _goal_pose(self):
        x = 0.0
        y = 6.0

        return (x, y)


