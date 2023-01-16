import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pybullet as p
import casadi as cs
from math import floor,sqrt


class VoxelGrid:
    def __init__(self, width, height, depth, resolution,min_dist,Wpen):
        # Store the dimensions of the grid in meters
        self.width = width
        self.height = height
        self.depth = depth
        self.min_dist=min_dist
        self.Wpen=Wpen
        self.objects=[]
        # Store the resolution of the voxels in meters
        self.resolution = resolution

        # Calculate the number of voxels in each dimension
        self.num_voxels_x = int(width / resolution)
        self.num_voxels_y = int(height / resolution)
        self.num_voxels_z = int(depth / resolution)

        # Create an empty 3D grid
        self.grid = np.zeros((self.num_voxels_x, self.num_voxels_y, self.num_voxels_z), dtype=np.int8)
        
    # def set_occupied(self, x, y, z):
    #     # Mark the voxel at the given coordinates as occupied
    #     self.grid[int(x / self.resolution), int(y / self.resolution), int(z / self.resolution)] = 1

    def set_occupied(self, x, y, z):
        # Mark the voxel at the given coordinates as occupied
        self.grid[x, y, z] = 1
    def set_free(self, x, y, z):
        # Mark the voxel at the given coordinates as free
        #self.grid[int(x / self.resolution), int(y / self.resolution), int(z / self.resolution)] = 0
        self.grid[x, y, z] = 0




    def is_occupied(self, x, y, z):
        # Return True if the voxel at the given coordinates is occupied, False otherwise

        return self.grid[int(x / self.resolution), int(y / self.resolution), int(z / self.resolution)] == 1

    def get_voxel_index(self, x, y, z):
        return int(x / self.resolution), int(y / self.resolution), int(z / self.resolution)

    def apply_boudary_conditions(self,i,j,k):
        max_i,max_j,max_k = self.num_voxels_x,self.num_voxels_y ,self.num_voxels_z  
        if i>(max_i-1):
            i =  max_i-1
        if j>(max_j-1):
            j=max_j-1
        if k>(max_k-1):
            k=max_k-1
        
        if i<0:
            i = 0
        if j<0:
            j=0
        if k<0:
            k=0
        return i,j,k

    def voxel_constraint(self,x,y,z):
        voxel_size = self.resolution
        print(x,y,z)
        print("we zitten in deze functie")
        min_dist=self.min_dist
        Wpen=self.Wpen
        i = int(x / self.resolution)
        j = int(y / self.resolution)
        k = int(z / self.resolution)
        penalty=0
        if self.is_occupied(x,y,z):
            print(1)
            return 0

        else:
            # find nearest occupied voxel
            for j_offset in range(-30,30,1):
                for i_offset in range(-30,30,1):
                    for k_offset in range(-30, 30, 1):
                        i_o = i + i_offset
                        j_o = j + j_offset
                        k_o = k + k_offset
                        i_o,j_o,k_o = self.apply_boudary_conditions(i_o,j_o,k_o)
                        if self.grid[i_o, j_o, k_o]:
                            dist = np.linalg.norm([i_offset * voxel_size, j_offset * voxel_size, k_offset * voxel_size])
                            if dist < min_dist:
                                min_dist=dist
                                penalty += Wpen/ (min_dist**2) 
            return penalty
    


def add_obstacles_to_grid(grid, obstacles):
    for obstacle in obstacles:
        x, y, z = obstacle.position
        i, j, k = grid.get_voxel_index(x, y, z)
        grid.set_occupied(i, j, k)


def add_torus_to_grid(grid, torus_id, resolution):
    # Get the dimensions and position of the torus
    aabb_min, aabb_max = p.getAABB(torus_id)
    x_min, y_min, z_min = aabb_min
    x_max, y_max, z_max = aabb_max
    x_center = (x_min + x_max) / 2
    y_center = (y_min + y_max) / 2
    z_center = (z_min + z_max) / 2
    x_radius = (x_max - x_min) / 2
    y_radius = (y_max - y_min) / 2
    z_radius = (z_max - z_min) / 2

    # Calculate the voxel indices for the free voxels inside the torus
    for i in range(grid.num_voxels_x):
        for j in range(grid.num_voxels_y):
            for k in range(grid.num_voxels_z):
                x = i * resolution + x_min
                y = j * resolution + y_min
                z = k * resolution + z_min
                r = np.sqrt((x - x_center)**2 + (y - y_center)**2)
                if r < x_radius and z < z_radius:

                    grid.set_free(i, j, k)




def add_obstacles_to_grid3(grid, obstacles,centers):
    print(obstacles)
    count = 0
    for obstacle_id in obstacles:
        # Get the AABB (axis-aligned bounding box) of the obstacle
        aabb_min, aabb_max = p.getAABB(obstacle_id)
        
        object_name = p.getBodyInfo(obstacle_id)[0]  

        print(object_name,count)
        if object_name == b'plane' or object_name == b'planeLink':
            continue


        if object_name == b'cf2' or object_name == b'base_link':  
            continue
        d = sqrt((aabb_max[0]-aabb_min[0])**2 + (aabb_max[1]-aabb_min[1])**2 + (aabb_max[2]-aabb_min[2])**2)
        info = {"d_obs":d,"d_robot":2*(0.0397+2.31348e-2),"x_center":centers["x"][count] ,"y_center":centers["y"][count],"z_center":centers["z"][count] }
        grid.objects.append(info)
        count+=1

        print(aabb_min,aabb_max,object_name)

        # Iterate over all voxels in the AABB
        for x in range(int(aabb_min[0] / grid.resolution), int(aabb_max[0] / grid.resolution) + 1):
            for y in range(int(aabb_min[1] / grid.resolution), int(aabb_max[1] / grid.resolution) + 1):
                for z in range(int(aabb_min[2] / grid.resolution), int(aabb_max[2] / grid.resolution) + 1):
                    # Mark the voxel as occupied
                    grid.set_occupied(x, y, z)
        # if object_name==b"torus":
        #     print("In de torus function")
        #     add_torus_to_grid(grid, obstacle_id, grid.resolution)
        print("created ")

def visualize_voxel_grid(voxel_grid,dense=False,xlim=10,ylim=10,zlim=10):
    # Create a figure and a 3D axis
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    print("Visualizing the grid")
    # Plot the occupied voxels as cubes
    for x in range(voxel_grid.num_voxels_x):
        for y in range(voxel_grid.num_voxels_y):
            for z in range(voxel_grid.num_voxels_z):
                x_pos = x * voxel_grid.resolution + voxel_grid.resolution / 2
                y_pos = y * voxel_grid.resolution + voxel_grid.resolution / 2
                z_pos = z * voxel_grid.resolution + voxel_grid.resolution / 2
                if voxel_grid.is_occupied(x_pos, y_pos, z_pos):
                    ax.scatter(x_pos, y_pos, z_pos, c='k', marker='s', s=1)
                elif dense==True:
                    ax.scatter(x_pos, y_pos, z_pos, c='r', marker='s', s=1)

    # Set the axis labels and show the plot
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim([0,xlim])
    ax.set_ylim([0,ylim])
    ax.set_zlim([0,zlim])

    plt.show()

# Define a function that takes in the symbolic variables and returns the penalty value
def voxel_constraint_expression(x, y, z, voxelgrid):
    voxel_size=voxelgrid.resolution
    min_dist = voxelgrid.min_dist
    Wpen = voxelgrid.Wpen
    i = cs.floor(x / voxelgrid.resolution)
    j = cs.floor(y / voxelgrid.resolution)
    k = cs.floor(z / voxelgrid.resolution)
    penalty = 0
    if 1==2:
        print(1)
    else:
        # find nearest occupied voxel
        for j_offset in [-1, 0, 1]:
            for i_offset in [-1, 0, 1]:
                for k_offset in [-1, 0, 1]:
                    i_o = i + i_offset
                    j_o = j + j_offset
                    k_o = k + k_offset
                    #i_o, j_o, k_o = apply_boudary_conditions(i_o, j_o, k_o)
                    #if voxel_grid[i_o, j_o, k_o]:
                    dist = cs.norm_2([i_offset * voxel_size, j_offset * voxel_size, k_offset * voxel_size])
                    if dist < min_dist:
                        min_dist = dist
                        penalty = Wpen / (min_dist ** 2)
        return penalty






# start = np.array([0, 0, 0])
# goal = np.array([1, 1, 1])

# voxel_grid = VoxelGrid(1, 1, 1, 0.1)

# # # Add some occupied voxels to the grid
# voxel_grid.set_occupied(0.5, 0.5, 0.5)
# voxel_grid.set_occupied(0.3, 0.2, 0.8)
# visualize_voxel_grid(voxel_grid,)