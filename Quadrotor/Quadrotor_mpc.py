"""3D quadrotor example script.

Example:

    $ python3 3d_quad.py --overrides ./3d_quad.yaml

"""
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
#from quadrotor_utils import PIDController
from MPC import *

from Quadrotor import *
#from results import *
import imageio
import imageio_ffmpeg
import pickle

def main():
    """The main function creating, running, and closing an environment.

    """
    # Set iterations and episode counter.
    num_episodes = 1
    ITERATIONS = int(100000)
    # Start a timer.
    START = time.time()
    # Create an environment
    CONFIG_FACTORY = ConfigFactory()
    config = CONFIG_FACTORY.merge()
    env_func = partial(make,
                       config.task,
                       seed=config.seed,
                       **config.quadrotor_config
                       )

    #Test waypoints
    #waypoints = np.array([(0, 0, 0.1), (0, .5, .5), (0.7, 0.1, 0.6), (1.0, 1.0, 1.0), (1.3, 1.5, 1.2)])
    
    #Train waypoints
    waypoints = np.array([(0, 0, 0.1), (0.3, .6, .7), (0.3, 1.0, 0.2), (1.0, 1.0, 1.0), (2.0, 2.0, 2.0)])


    # Q 
    Q = np.zeros(12)
    Q[0],Q[2],Q[4] =15,15,15
    Q[1],Q[3],Q[5] = 0.1,0.1,0.1
    Q[9],Q[10],Q[11] = 0.001,0.001,0.001  
    obstacles =True 
    ctrl =MPC(env_func=env_func,waypoints=waypoints,deg=6,intermediatepoints=10,obstacles=obstacles,q_mpc=Q,r_mpc=[1],horizon=10,soft_constraints=False) 
    ctrl.reset()
    env = ctrl.env
    #Creates trajectory reference plot in environment
    for i in range(10, ctrl.traj.shape[1], 10):
        p.addUserDebugLine(lineFromXYZ=[ctrl.traj[0][i-10], ctrl.traj[2][i-10], ctrl.traj[4][i-10]],
                           lineToXYZ=[ctrl.traj[0][i], ctrl.traj[2][i], ctrl.traj[4][i]],
                           lineColorRGB=[1, 0, 0],
                           physicsClientId=env.PYB_CLIENT)
    obs,info,frame = ctrl.run(waarde=0)
    # print(les)
    i = 0
    try:
        while i < ITERATIONS and ctrl.done != True:
            obs,info, frame = ctrl.run(obss=True,obs=obs,info=info, waarde=i+ctrl.T)
            i += ctrl.T


            # if i == 10*ctrl.T:

            #     break
    except Exception:
        print("jem")

    with open('results_dict_con3_more_obs2.pkl', 'wb') as f:
        pickle.dump(ctrl.results_dict,f)

    
    return 0


if __name__ == "__main__":
    main()