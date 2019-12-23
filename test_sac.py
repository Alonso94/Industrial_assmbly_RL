from env.env_real_sac import rozum_real
env=rozum_real()

import gym
import numpy as np

import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'

import tensorflow as tf
tf.compat.v1.logging.set_verbosity(tf.compat.v1.logging.ERROR)

from tensorflow.python.util import deprecation
deprecation._PRINT_DEPRECATION_WARNINGS = False


from stable_baselines.sac.policies import MlpPolicy
from stable_baselines import SAC
from stable_baselines import results_plotter

os.chdir("/")
model = SAC.load("/home/ali/Industrial_assmbly_RL/sac_rozum_new(2).zip",env=env)
# print(model.get_parameters())
# model.learn(total_timesteps=1000, log_interval=10)#,tb_log_name="stage2")
# model.save("sac_rozum2")
print(model.action_space)
print(model.action_space)
print("\n After training \n")
obs = env.reset()
for i in range(200):
    action, _states = model.predict(obs)
    obs, reward, done, info = env.step(action)
    print(reward)
    if done:
        env.reset()