import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
# import pybullet as p
# import casadi as cs
from math import floor,sqrt,pi
# from linear_mpc import * 
# from MPC import *
# from Voxel import *
# from Quadrotor import *
import pandas as pd
import scienceplots

plt.style.use(['science','ieee'])

#Function plots trajectory error of the drone.
def plot_trajectory_error(path_error):
    fig, ax = plt.subplots()
    path_error = np.array(path_error)
    x = path_error[:,0]
    y = path_error[:,2]
    z = path_error[:,4]
    err = np.sqrt(x**2+y**2+z**2)
    t = np.linspace(0,20,len(err))
    #Convert to pandas dataframe
    df = pd.DataFrame(err)

    # Apply a rolling mean with a window size of n
    n = 50 # window size
    err_smooth = df.rolling(n).mean()*1000
    
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Trajectory error [mm]")
    ax.autoscale(tight=True)

    ax.plot(t,err_smooth)
    fig.savefig('trajec_error.png')


def plot_speed_error(results_trajec,states):
    ref_speeds = np.vstack((np.zeros((1,3)),results_trajec[[1,3,5],: ].T))
    speeds = states[:,[1,3,5]]
    euc_error=  np.linalg.norm(speeds-ref_speeds,axis=1)
    t = np.linspace(0,20,len(euc_error))
    
    fig, ax = plt.subplots()
    ax.plot(t,euc_error,color="black")
        
    ax.autoscale(tight=True)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel(r"Speed error [m $s^{-1}$]")
    fig.savefig('velerrror.png')


def plot_trajectory_with_obstacles(states,trajec):
    static_obstacles = {'2': {'d_obs': 0.12124355652982141, 'd_robot': 0.1256696, 'x_center': 0.09, 'y_center': 0.18, 'z_center': 0.42000000000000004}, '3': {'d_obs': 0.12124355652982138, 'd_robot': 0.1256696, 'x_center': 0.3, 'y_center': 0.84, 'z_center': 0.4}, '4': {'d_obs': 0.12124355652982144, 'd_robot': 0.1256696, 'x_center': 0.58, 'y_center': 1.0, 'z_center': 0.52}, '5': {'d_obs': 0.12124355652982152, 'd_robot': 0.1256696, 'x_center': 1.5, 'y_center': 1.5, 'z_center': 1.5}}
    goals_obstacles = [{'d_obs': 0.2591888429401223, 'd_robot': 0.1256696, 'x_center': 1.0, 'y_center': 1.0, 'z_center': 1.0}, {'d_obs': 0.2591888429401223, 'd_robot': 0.1256696, 'x_center': 1.8, 'y_center': 1.8, 'z_center': 1.8}, {'d_obs': 0.25918884294012223, 'd_robot': 0.1256696, 'x_center': 0.3, 'y_center': 0.6, 'z_center': 0.7}, {'d_obs': 0.26434958020347665, 'd_robot': 0.1256696, 'x_center': 0.3, 'y_center': 1.0, 'z_center': 0.2}]
        
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    x,y,z = states[:,0],states[:,2],states[:,4]
    x_ref,y_ref,z_ref = trajec[0,:],trajec[2,:],trajec[4,:]
    
    ax.plot(x,y,z,color="black",label="Trajectory",linestyle="-")
    ax.plot(x_ref,y_ref,z_ref,color="Orange",label="Reference",linestyle="--")
    
    static_keys= list(static_obstacles.keys())
    count=0
    for key in static_keys:
        count+=1
        obstac = static_obstacles[key]
        x_cent,y_cent,z_cent = obstac["x_center"],obstac["y_center"],obstac["z_center"]
        if count==len(static_keys):
            ax.scatter(x_cent,y_cent,z_cent,marker="x",color ="red",label="Obstacle")
        else:
            ax.scatter(x_cent,y_cent,z_cent,marker="x",color ="red")

    count=0 
    for obstac in goals_obstacles:
        count+=1
        x_cent,y_cent,z_cent = obstac["x_center"],obstac["y_center"],obstac["z_center"]
        if count==len(goals_obstacles):
            ax.scatter(x_cent,y_cent,z_cent,marker="o",color ="Green",label="Goal")
        else:
            ax.scatter(x_cent,y_cent,z_cent,marker="o",color ="Green")

    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    ax.legend()
    fig.savefig('Trajec_with_obstac.png')

def plot_total_cost(cost):
    fig, ax = plt.subplots()
    cost = np.array(cost)

    #Convert to pandas dataframe
    df = pd.DataFrame(cost)

    # Apply a rolling mean with a window size of n
    n = 20 # window size
    cost_smooth = df.rolling(n).mean()

    ax.set_xlabel("Time [s] ")
    ax.set_ylabel("Cost [-]")
    ax.autoscale(tight=True)

    t = np.linspace(0,20,len(cost_smooth))
    ax.plot(t,cost_smooth,color="black")
    fig.savefig('cost.png')
def plot_cost_3d(states,cost):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    x,y,z = states[:,0],states[:,2],states[:,4]
    costf =np.array(cost)
    
    img = ax.scatter(x[1:], y[1:], z[1:], c=costf)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    fig.colorbar(img)
    plt.show()
    ax.view_init(azim=-60-80)
    fig.savefig('Costfunction3d.png')
def plot_roll_pitch_yaw(states,plotyaw=False,output=None):
    fig, ax = plt.subplots()
    states = states*180/pi
    roll,pitch,yaw= states[:,6],states[:,7],states[:,8]  #phi,theta,psi 
    print(len(roll))
    t = np.linspace(0,20,len(roll))
    print(len(t))
    ax.plot(t,roll,label = r"$\phi$",linestyle="-",color="orange")
    ax.plot(t,pitch, label=r"$\theta$",linestyle="-",color="green")
    if plotyaw:
        ax.plot(t,yaw, label=r"$\psi$",linestyle="-",color="black")
    
    ax.legend()
    ax.autoscale(tight=True)
    
    ax.set_xlabel("Time [s] ")
    ax.set_ylabel(r"Angle [deg]")
    if output==None:
        if plotyaw:
            fig.savefig('roll_pitch_yaw.png')
        else:
            fig.savefig('roll_pitch.png')
    else:
        if plotyaw:
            fig.savefig(output)
        else:
            fig.savefig(output)

def plot_roll_pitch_yawrate(states,plotyawrate=False,output=None):
    fig, ax = plt.subplots()
    states = states*180/pi

    rolld,pitchd,yawd= states[:,9],states[:,10],states[:,11]  #phid,thetad,psid
    t = np.linspace(0,20,len(rolld))

    ax.plot(t,rolld,label = r"$\dot{\phi}$",linestyle="-",color="orange")
    ax.plot(t,pitchd, label=r"$\dot{\theta}$",linestyle="-",color="green")
    if plotyawrate:
        ax.plot(t,yawd, label=r"$\dot{\psi}$",linestyle="-",color="black")
    
    ax.legend()
    ax.autoscale(tight=True)
    
    ax.set_xlabel("Time [s] ")
    ax.set_ylabel(r"Anglerate [deg s$^{-1}$]")
    if output==None:
        if plotyawrate:
            fig.savefig('rateroll_pitch_yaw.png')
        else:
            fig.savefig('rateroll_pitch.png')
    else:
        if plotyawrate:
            fig.savefig(output)
        else:
            fig.savefig(output)



    
    
    