from gymnasium.envs.registration import register

register(
     id="ros-gazebo-v0",
     entry_point='ros_gazebo_env:RosGazebo',
     max_episode_steps=3000,
)
