import gymnasium as gym
from stable_baselines3 import PPO
from gymnasium.envs.registration import register
from stable_baselines3.common.callbacks import CheckpointCallback

register(
     id="ros-gazebo-v0",
     entry_point='ros_gazebo_env:RosGazebo',
     max_episode_steps=50,
)

env = gym.make('ros-gazebo-v0')

# Stop training when the model reaches the reward threshold
checkpoint_callback = CheckpointCallback(
  save_freq=2000,
  save_path="./logs/",
  name_prefix="rl_model",
  save_replay_buffer=True,
  save_vecnormalize=True,
)

model = PPO('MultiInputPolicy', env, verbose=1, ent_coef=0.1, gamma=0.5, n_steps=50, batch_size=50)
model.learn(total_timesteps=50000, log_interval=1, progress_bar=True, callback=checkpoint_callback)
model.save("PPO_ros_gazebo_parameters_adjusted")

env.close()