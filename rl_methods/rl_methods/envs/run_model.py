import gymnasium as gym
from stable_baselines3 import PPO
from gymnasium.envs.registration import register

register(
     id="ros-gazebo-v0",
     entry_point='ros_gazebo_env:RosGazebo',
     max_episode_steps=3000,
)

model = PPO.load("./logs/rl_model_38000_steps.zip")

env = gym.make('ros-gazebo-v0')

obs, info = env.reset()
while True:
    action, _states = model.predict(obs, deterministic=True)
    obs, reward, terminated, truncated, info = env.step(action)
    if terminated or truncated:
        obs, info = env.reset()
