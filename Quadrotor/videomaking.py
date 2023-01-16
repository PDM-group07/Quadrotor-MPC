import os
import time
import yaml
import inspect
import numpy as np
import pybullet as p
import casadi as cs
import matplotlib.pyplot as plt
from functools import partial
from safe_control_gym.utils.utils import str2bool
from safe_control_gym.utils.configuration import ConfigFactory
from safe_control_gym.utils.registration import make
from MPC import *
from Quadrotor import *
#from results import *
import pickle
from create_obstacles import *


with open('results_dict_con3_more_obs2.pkl', 'rb') as f:
    results_dict = pickle.load(f)
actions = results_dict['action']


START = time.time()
# Create an environment
CONFIG_FACTORY = ConfigFactory()
config = CONFIG_FACTORY.merge()
waypoints = np.array([(0, 0, 0.1), (0.3, .6, .7), (0.3, 1.0, 0.2), (1.0, 1.0, 1.0), (2.0, 2.0, 2.0)])


env_func = partial(make,
                    config.task,
                    seed=config.seed,
                    **config.quadrotor_config
                    )
env = env_func()
traj,Goals_indx =create_trajec(waypoints,env.EPISODE_LEN_SEC,env.CTRL_FREQ,deg=0,intermediatepoints=0) #self.env.X_GOAL.T
env.X_GOAL = traj.T
env.reset()


Obs = Obstacles()
Obs.create_static_obstacles(env,waypoints)
Obs.static_obstacles_info()
Obs.create_dynamic_obstacle(env,waypoints)
Obs.goals_obstacles_info()
Obs.initiator(env)

for action in actions:
    env.step(action)
    time.sleep(20/1200)
