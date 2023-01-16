import numpy as np
from scipy.interpolate import CubicSpline

from math import ceil


def fit_waypoints(waypoints, deg, num_points):
    # Fit x, y, and z separately
    fit_x = np.polyfit(np.arange(waypoints.shape[0]), waypoints[:,0], deg)
    fit_y = np.polyfit(np.arange(waypoints.shape[0]), waypoints[:,1], deg)
    fit_z = np.polyfit(np.arange(waypoints.shape[0]), waypoints[:,2], deg)

    # Create a polynomial function for each dimension
    fx = np.poly1d(fit_x)
    fy = np.poly1d(fit_y)
    fz = np.poly1d(fit_z)

    # Generate new x, y, and z values using the polynomial function
    t_scaled = np.linspace(0, waypoints.shape[0]-1, num_points)
    x_poly = fx(t_scaled)
    y_poly = fy(t_scaled)
    z_poly = fz(t_scaled)

    # Concatenate x, y, and z into a single array
    waypoints_poly = np.concatenate((x_poly[:, np.newaxis], y_poly[:, np.newaxis], z_poly[:, np.newaxis]), axis=1)
    return waypoints_poly
    
def create_trajec(waypoints,epslen,ctrlfreq,deg=6,intermediatepoints=10):
    totalpoints = ctrlfreq*epslen
    Total_distance = (sum(np.sum(waypoints**2,axis=0)))**0.5
    Velocity= Total_distance/epslen

    Rel_dists = np.zeros(len(waypoints)-1 )
    rcs = np.zeros((len(waypoints)-1,3))

    indices = []
    index_start=0
    for i in range(1,len(waypoints)):
        prev_wp,wp = waypoints[i-1,:],waypoints[i,:]
        diffwp = wp-prev_wp
        
        reldist =  (np.sum(diffwp**2))**0.5 /  Total_distance
        points_to_add = ceil(intermediatepoints * reldist)    
        if i==1:
            waypoints_ = np.linspace(prev_wp,wp,num=points_to_add+2,endpoint=True)[:-1]
            waypoint_loc  = waypoints_ 

        elif i==len(waypoints)-1:
            waypoint_loc = np.linspace(prev_wp,wp,num=points_to_add+1,endpoint=True)
            waypoints_= np.concatenate((waypoints_,waypoint_loc))
        else:
            waypoint_loc = np.linspace(prev_wp,wp,num=points_to_add+2,endpoint=True)[:-1]

            waypoints_= np.concatenate((waypoints_,waypoint_loc))
        index_start+=int(len(waypoint_loc))
        indices.append(index_start-1)
    Trajec = np.zeros((12,totalpoints))
    Trajec[[0,2,4]] = (fit_waypoints(waypoints_, deg, totalpoints)).T
    

    indices = np.array(indices,dtype=np.int32)
    indices = np.ceil(indices/indices[-1] * totalpoints)    
    indices = [int(i) for i in indices]
    return Trajec,indices


def create_trajec(waypoints,epslen,ctrlfreq,deg=6,intermediatepoints=10):
    totalpoints = ctrlfreq*epslen
    Total_distance = (sum(np.sum(waypoints**2,axis=0)))**0.5
    Velocity= Total_distance/epslen


    Rel_dists = np.zeros(len(waypoints)-1 )
    rcs = np.zeros((len(waypoints)-1,3))
    for i in range(1,len(waypoints)):
        prev_wp,wp = waypoints[i-1,:],waypoints[i,:]
        diffwp = wp-prev_wp

        reldist =  (np.sum(diffwp**2))**0.5 /  Total_distance

        rc = prev_wp
        Rel_dists[i-1]=reldist
        rcs[i-1] = wp-prev_wp

    Points = np.floor(Rel_dists*totalpoints)
    Points[-1] += totalpoints -np.sum(Points) 

    Trajec = np.zeros((12,totalpoints))
    start = 0
    end=0
    indices = []
    index_start = 0
    for i in range(1,len(waypoints)):
        prev_wp,wp = waypoints[i-1],waypoints[i]
        speed = Velocity * rcs[i-1]/(np.sum(rcs[i-1]**2)**0.5)
        if i==1:
            local_traj = np.linspace(prev_wp,wp,num=int(Points[i-1]),endpoint=True)
        else:
            local_traj = np.linspace(prev_wp+rcs[i-1]/int(Points[i-1]),wp,num=int(Points[i-1]),endpoint=True)
        end += int(Points[i-1])

        Trajec[[0,2,4],start:end] = local_traj.T
        Trajec[[1,3,5],start:end] = (speed*np.ones((int(Points[i-1]),3 ))).T
        start = end
        index_start+=int(len(local_traj))
        indices.append(index_start-1)
    
    indices = np.array(indices,dtype=np.int32)
    indices = np.ceil(indices/indices[-1] * totalpoints)    
    indices = [int(i) for i in indices]            
    # trajs =  fit_traj(traj,deg=deg)
    # Trajec[[0,2,4],:] = trajs.T
    return  Trajec,indices


