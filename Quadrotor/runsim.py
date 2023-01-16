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

from safe_control_gym.utils.utils import str2bool
from safe_control_gym.utils.configuration import ConfigFactory
from safe_control_gym.utils.registration import make
from safe_control_gym.envs.gym_pybullet_drones.quadrotor_utils import PIDController
from linear_mpc import *
from Quad_env import *
def sync(i, start_time, timestep):
    """Syncs the stepped simulation with the wall-clock.

    Function `sync` calls time.sleep() to pause a for-loop
    running faster than the expected timestep.

    Parameters
    ----------
    i : int
        Current simulation iteration.
    start_time : timestamp
        Timestamp of the simulation start.
    timestep : float
        Desired, wall-clock step of the simulation's rendering.

    """
    if timestep > .04 or i%(int(1/(24*timestep))) == 0:
        elapsed = time.time() - start_time
        if elapsed < (i*timestep):
            time.sleep(timestep*i - elapsed)


def main():
    """The main function creating, running, and closing an environment.

    """
    # Set iterations and episode counter.
    num_episodes = 1
    ITERATIONS = int(1000)
    # Start a timer.
    START = time.time()

    # Create an environment
    num_drones = 1
    INIT_XYZS = np.array([[0, 0, 0.1] for i in range(num_drones)])
    INIT_RPYS = np.array([[0, 0,  i * (np.pi/2)/num_drones] for i in range(num_drones)])
    physics = Physics("pyb")
    drone=DroneModel("cf2x") 
    record_video= False
    obstacles = True    
    user_debug_gui=False
    gui =True
    AGGR_PHY_STEPS=True

    # CONFIG_FACTORY = ConfigFactory()
    # config = CONFIG_FACTORY.merge()
    simulation_freq_hz= 240



    env= BaseAviary(drone_model=drone,
                         num_drones=num_drones,
                         initial_xyzs=INIT_XYZS,
                         initial_rpys=INIT_RPYS,
                         physics=physics,
                         neighbourhood_radius=10,
                         freq=simulation_freq_hz,
                         aggregate_phy_steps=AGGR_PHY_STEPS,
                         gui=gui,
                         record=record_video,
                         obstacles=obstacles,
                         user_debug_gui=user_debug_gui
                         )
    # Controller
    ctrl = LinearMPC(env=env)
    env._reset_simulation()
    # Reset the environment, obtain and print the initial observations.

    # Curve fitting with waypoints.
    waypoints = np.array([(0, 0, 0), (0.2, 0.5, 0.5), (0.5, 0.1, 0.6), (1, 1, 1), (1.3, 1, 1.2)])
    deg = 6
    t = np.arange(waypoints.shape[0])
    fit_x = np.polyfit(t, waypoints[:,0], deg)
    fit_y = np.polyfit(t, waypoints[:,1], deg)
    fit_z = np.polyfit(t, waypoints[:,2], deg)
    fx = np.poly1d(fit_x)
    fy = np.poly1d(fit_y)
    fz = np.poly1d(fit_z)
    t_scaled = np.linspace(t[0], t[-1], env.EPISODE_LEN_SEC*env.CTRL_FREQ)
    ref_x = fx(t_scaled)
    ref_y = fy(t_scaled)
    ref_z = fz(t_scaled)

    for i in range(10, ref_x.shape[0], 10):
        p.addUserDebugLine(lineFromXYZ=[ref_x[i-10], ref_y[i-10], ref_z[i-10]],
                           lineToXYZ=[ref_x[i], ref_y[i], ref_z[i]],
                           lineColorRGB=[1, 0, 0],
                           physicsClientId=env.PYB_CLIENT)

    for point in waypoints:
        p.loadURDF(os.path.join(env.URDF_DIR, "gate.urdf"),
                   [point[0], point[1], point[2]-0.05],
                   p.getQuaternionFromEuler([0,0,0]),
                   physicsClientId=env.PYB_CLIENT)

    # Run an experiment.
    action = np.array([0, 0, 0, 0])

    for i in range(ITERATIONS):

        # Step by keyboard input
        # _ = input('Press any key to continue.')
        obs, reward, done, info = env._advance_simulation(action)
        obs= obs['0']["state"]
        rpms,poserr,rperror = ctrl.compute_control(control_timestep=env.CTRL_TIMESTEP,
                    cur_pos=obs[0:3],#np.array([obs[0],obs[1],obs[2]]),
                    cur_quat=obs[3:7],##np.array([obs[3],obs[4],obs[5],obs[6]]),
                    cur_vel=obs[10:13],#np.array([obs[10],obs[11],obs[12]]),
                    cur_ang_vel=obs[13:16],#np.array([obs[13],obs[14],obs[15]]),
                    target_pos=np.array([ref_x[i], ref_y[i], ref_z[i]]),
                    target_vel=np.zeros(3)
                    )
        action = rpms

        # If an episode is complete, reset the environment.
        if done:
            num_episodes += 1
            new_initial_obs, new_initial_info = env._reset_simulation()
            print(str(num_episodes)+'-th reset.', 7)
            print('Reset obs' + str(new_initial_obs), 2)
            print('Reset info' + str(new_initial_info), 0)

    # Close the environment and print timing statistics.
    env.close()
    elapsed_sec = time.time() - START
    out = str("\n{:d} iterations (@{:d}Hz) and {:d} episodes in {:.2f} seconds, i.e. {:.2f} steps/sec for a {:.2f}x speedup.\n\n"
          .format(ITERATIONS, env.CTRL_FREQ, num_episodes, elapsed_sec, ITERATIONS/elapsed_sec, (ITERATIONS*env.CTRL_TIMESTEP)/elapsed_sec))
    print(out)

main()
if __name__ == "    ":
    main()
