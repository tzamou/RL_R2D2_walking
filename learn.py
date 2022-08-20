from stable_baselines3 import PPO
from envstep_1 import Step1
import matplotlib.pyplot as plt
import gym
import numpy as np


# Parallel environments
env = Step1()

model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=500000)
plt.plot(env.rewardlst)
plt.show()
model.save("ppo_r2d2")

del model # remove to demonstrate saving and loading

model = PPO.load("ppo_r2d2")

obs = env.reset()
# while True:
#     action, _states = model.predict(obs)
#     obs, rewards, dones, info = env.step(action)
#     if dones:
#         env.reset()
#     env.render()

