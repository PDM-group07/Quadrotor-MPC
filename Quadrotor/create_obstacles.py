



import pybullet as p
import os 
import numpy as np
from math import sqrt,pi
class Obstacles:
    def __init__(self):
        self.static_centers = {"x":[],"y":[],"z":[]}
        self.dynamic_centers =  {"x":[],"y":[],"z":[]}
        self.dist_add = 0
        self.dynamic_objects ={}
        self.static_objects={}
        self.goals_objects=[] 

        self.dynamic_obstacles = []
        self.static_obstacles= []
        self.goals_obstacles = []
        
        self.rings_dist_to_add= [[0,0,0] ,[0,0,0],[0,0,0],[0,0,0]]

        self.rings_eulers = [ [90*pi/180,0,0],[90*pi/180,0,-45*pi/180],[90*pi/180,0,-90*pi/180],[90*pi/180,0,-90*pi/180]          ]


        self.goal_scaler = .0024
        self.R_goal=70*self.goal_scaler
        self.shift= [0, 0 , 0, 0.2]
        self.Trans_goals = []
        self.point_goals_torus = []
        self.goal_rotations = []

    

    def create_goals(self,env,waypoints):
        for i in range(1,len(waypoints)):
            shift = self.shift[i-1]
            wp_prev = waypoints[i-1]
            wp = waypoints[i]
            dist_ = (wp-wp_prev)*shift



            orn = np.array(self.rings_eulers)[i-1] 
            alpha,beta,gamma = orn[0],orn[1],orn[2]
            Rx = np.array([[1, 0, 0],
                        [0, np.cos(alpha), -np.sin(alpha)],
                        [0, np.sin(alpha), np.cos(alpha)]])

            Ry = np.array([[np.cos(beta), 0, np.sin(beta)],
              [0, 1, 0],
              [-np.sin(beta), 0, np.cos(beta)]])

            Rz = np.array([[np.cos(gamma), -np.sin(gamma), 0],
                        [np.sin(gamma), np.cos(gamma), 0],
                        [0, 0, 1]])

            R = Rz@Ry@Rx
            self.goal_rotations.append( R)

            T = np.array([[R[0,0], R[0,1], R[0,2], waypoints[i][0]   ],    
                            [R[1,0], R[1,1], R[1,2] , waypoints[i][1]     ],
                            [R[2,0], R[2,1], R[2,2],  waypoints[i][2]  ],
                            [0, 0, 0, 1]])
            point_torus = np.array([0,0,shift,1])

            point_world = T@point_torus
            shifted_pos = point_world
            shifted_pos = wp-dist_


            T = np.array([[R[0,0], R[0,1], R[0,2], shifted_pos[0]   ],    
                            [R[1,0], R[1,1], R[1,2] , shifted_pos[1]     ],
                            [R[2,0], R[2,1], R[2,2],  shifted_pos[2]    ],
                            [0, 0, 0, 1]])
            
            R_torus = self.R_goal
            pis =np.linspace(0,2*pi,6,endpoint=False)
            xx = R_torus*np.cos(pis)
            yy = R_torus*np.sin(pis)
            zz = pis*0
            d = np.ones(len(zz))
            points_torus = np.array([xx,yy,zz,d])

            self.point_goals_torus.append((T@points_torus)[:3,:])
            self.Trans_goals.append(T)
            shifted_pos = wp-dist_
            torus = p.loadURDF(os.path.join(env.URDF_DIR, "torus.urdf"),
            [shifted_pos[0], shifted_pos[1], shifted_pos[2]   ],
            p.getQuaternionFromEuler(self.rings_eulers[i-1]),
            physicsClientId=env.PYB_CLIENT)
            p.setCollisionFilterGroupMask(torus, -1, 0, 0)


    def create_static_obstacles(self,env,waypoints):
        dist_add =self.dist_add
        pos = waypoints[0] + (waypoints[1]-waypoints[0])*0.3
        pos[2] = 1.5*pos[2]


        self.static_centers["x"].append(pos[0])
        self.static_centers["y"].append(pos[1])
        self.static_centers["z"].append(pos[2])

        # Set the position and orientation of the cube
        position = pos
        orientation = p.getQuaternionFromEuler([0, 0, 0])

        # Create a visual shape for the cube
        visualShapeId = p.createVisualShape(shapeType=p.GEOM_BOX,
                                            halfExtents=[0.07, 0.07, 0.07],
                                            rgbaColor=[1, 0, 0, 1])  # red color

        # Create a collision shape for the cube
        collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                                halfExtents=[0.07, 0.07, 0.07])

        # Create the cube using the visual and collision shapes
   
        cube = p.createMultiBody(baseMass=0,
                                baseInertialFramePosition=[0, 0, 0],
                                baseCollisionShapeIndex=collisionShapeId,
                                baseVisualShapeIndex=visualShapeId,
                                basePosition=position,
                                baseOrientation=orientation)
        pos = waypoints[1] + (waypoints[2]-waypoints[1])*0.6


        self.static_centers["x"].append(pos[0])
        self.static_centers["y"].append(pos[1])
        self.static_centers["z"].append(pos[2])

        # Set the position and orientation of the cube
        position = pos
        orientation = p.getQuaternionFromEuler([0, 0, 0])

        # Create a visual shape for the cube
        visualShapeId = p.createVisualShape(shapeType=p.GEOM_BOX,
                                            halfExtents=[0.07, 0.07, 0.07],
                                            rgbaColor=[1, 0, 0, 1])  # red color

        # Create a collision shape for the cube
        collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                                halfExtents=[0.07, 0.07, 0.07])

        # Create the cube using the visual and collision shapes
   
        cube = p.createMultiBody(baseMass=0,
                                baseInertialFramePosition=[0, 0, 0],
                                baseCollisionShapeIndex=collisionShapeId,
                                baseVisualShapeIndex=visualShapeId,
                                basePosition=position,
                                baseOrientation=orientation)



        pos = waypoints[2] + (waypoints[3]-waypoints[2])*0.4


        self.static_centers["x"].append(pos[0])
        self.static_centers["y"].append(pos[1])
        self.static_centers["z"].append(pos[2])

        # Set the position and orientation of the cube
        position = pos
        orientation = p.getQuaternionFromEuler([0, 0, 0])

        # Create a visual shape for the cube
        visualShapeId = p.createVisualShape(shapeType=p.GEOM_BOX,
                                            halfExtents=[0.07, 0.07, 0.07],
                                            rgbaColor=[1, 0, 0, 1])  # red color

        # Create a collision shape for the cube
        collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                                halfExtents=[0.07, 0.07, 0.07])

        # Create the cube using the visual and collision shapes
   
        cube = p.createMultiBody(baseMass=0,
                                baseInertialFramePosition=[0, 0, 0],
                                baseCollisionShapeIndex=collisionShapeId,
                                baseVisualShapeIndex=visualShapeId,
                                basePosition=position,
                                baseOrientation=orientation)



        pos = waypoints[3] + (waypoints[4]-waypoints[3])*0.5


        self.static_centers["x"].append(pos[0])
        self.static_centers["y"].append(pos[1])
        self.static_centers["z"].append(pos[2])

        # Set the position and orientation of the cube
        position = pos
        orientation = p.getQuaternionFromEuler([0, 0, 0])

        # Create a visual shape for the cube
        visualShapeId = p.createVisualShape(shapeType=p.GEOM_BOX,
                                            halfExtents=[0.07, 0.07, 0.07],
                                            rgbaColor=[1, 0, 0, 1])  # red color

        # Create a collision shape for the cube
        collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                                halfExtents=[0.07, 0.07, 0.07])

        # Create the cube using the visual and collision shapes
   
        cube = p.createMultiBody(baseMass=0,
                                baseInertialFramePosition=[0, 0, 0],
                                baseCollisionShapeIndex=collisionShapeId,
                                baseVisualShapeIndex=visualShapeId,
                                basePosition=position,
                                baseOrientation=orientation)







                                
        self.static_obstacles= list(range(p.getNumBodies()))
        self.create_goals(env,waypoints)
        self.goals_obstacles  =  list(set( list(range(p.getNumBodies())  )).difference(set(self.static_obstacles)))


    def create_dynamic_obstacle(self,env,waypoints):   #Insert dynamic obstacle function here
        dist_add =self.dist_add
        position = [0.5*waypoints[1][0], 0.5*waypoints[1][1]-dist_add, 0.5*waypoints[1][2]-dist_add*0.5]
        orientation = p.getQuaternionFromEuler([0, 0, 0])

        # Create a visual shape for the cube
        visualShapeId = p.createVisualShape(shapeType=p.GEOM_BOX,
                                            halfExtents=[0.07, 0.07, 0.07],
                                            rgbaColor=[1, 0, 0, 1])  # red color

        # Create a collision shape for the cube
        collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                                halfExtents=[0.07, 0.07, 0.07])
        for i in range(1,len(waypoints)):
            position = [waypoints[i][0]+0.05, waypoints[i][1]-0.25, waypoints[i][2]-0.3]
            cube = p.createMultiBody(baseMass=0,
                                    baseInertialFramePosition=[0, 0, 0],
                                    baseCollisionShapeIndex=collisionShapeId,
                                    baseVisualShapeIndex=visualShapeId,
                                    basePosition=position,
                                    baseOrientation=orientation)
            p.setCollisionFilterGroupMask(cube, -1, 0, 0)
            initial_linear_velocity = self.initiator(env)

            p.resetBaseVelocity(cube, initial_linear_velocity)
            
            self.dynamic_centers["x"].append(position[0])
            self.dynamic_centers["y"].append(position[1])
            self.dynamic_centers["z"].append(position[2])
        self.dynamic_obstacles  =  list(set(list(range(p.getNumBodies()))).difference(set(self.static_obstacles)))   
        self.dynamic_obstacles  =  list(set(self.dynamic_obstacles ).difference(set(self.goals_obstacles)))






    def initiator(self, env):
        dyn_bodies = self.dynamic_obstacles
        centers = self.dynamic_centers
        x, y, z = env.state[0], env.state[2], env.state[4]
        initial_vel = [0, 0, 0]
        specific_id = []

        # set waypoints
        waypoints = [[1, 2, 3], [4, 5, 6], [7, 8, 9]]
        current_waypoint_index = 0
        speed = 0.05 # set speed for moving towards waypoints
        threshold = 0.05 # set distance threshold

        for i, obstacle_id in enumerate(dyn_bodies):
            object_name = p.getBodyInfo(obstacle_id)[0]  
            if object_name == b'plane' or object_name == b'planeLink':
                continue
            current_position = p.getBasePositionAndOrientation(obstacle_id)[0]
            # check distance to torus

            torus_center = np.array([self.goals_objects[i]["x_center"],self.goals_objects[i]["y_center"],self.goals_objects[i]["z_center"]])


            distance = sqrt((current_position[0]-torus_center[0])**2 + (current_position[1]-torus_center[1])**2 + (current_position[2]-torus_center[2])**2)
            
            if distance < threshold:
                p.resetBaseVelocity(obstacle_id, [0, 0, 0])
                continue
            waypoint = waypoints[current_waypoint_index]
            # calculate the direction vector to the waypoint
            direction_vector = [waypoint[0] - current_position[0] + obstacle_id  , waypoint[1] - current_position[1]+obstacle_id, waypoint[2] - current_position[2] +obstacle_id  ]
            norm = sqrt(direction_vector[0]**2 + direction_vector[1]**2 + direction_vector[2]**2)
            direction_vector = [direction_vector[0]/norm, direction_vector[1]/norm, direction_vector[2]/norm]
            # set the velocity of the obstacle to be in the direction of the waypoint
            p.resetBaseVelocity(obstacle_id, [direction_vector[0]*speed, direction_vector[1]*speed, direction_vector[2]*speed])
            # check if the obstacle has reached the waypoint and move to the next waypoint if so
            if abs(current_position[0] - waypoint[0]) < 0.1 and abs(current_position[1] - waypoint[1]) < 0.1 and abs(current_position[2] - waypoint[2]) < 0.1:
                current_waypoint_index = (current_waypoint_index + 1) % len(waypoints)

    def static_obstacles_info(self):
        count = 0
        #centers =self.static_centers
        for obstacle_id in self.static_obstacles:
            # Get the AABB (axis-aligned bounding box) of the obstacle
            aabb_min, aabb_max = p.getAABB(obstacle_id)
            
            object_name = p.getBodyInfo(obstacle_id)[0]  

            if object_name == b'plane' or object_name == b'planeLink':
                continue
            if object_name == b'cf2' or object_name == b'base_link':  
                continue
            d = sqrt((aabb_max[0]-aabb_min[0])**2 + (aabb_max[1]-aabb_min[1])**2 + (aabb_max[2]-aabb_min[2])**2)/2
            pos= p.getBasePositionAndOrientation(obstacle_id)[0]

            info = {"d_obs":d,"d_robot":2*(0.0397+2.31348e-2),"x_center":pos[0],"y_center":pos[1],"z_center":pos[2] }
            self.static_objects[str(obstacle_id)] = info
            count+=1

    def dynamic_obstacles_info(self):
        dyn_bodies = self.dynamic_obstacles 
        for obstacle_id in dyn_bodies:
            aabb_min, aabb_max = p.getAABB(obstacle_id)
            object_name = p.getBodyInfo(obstacle_id)[0]  
            if object_name == b'plane' or object_name == b'planeLink':
                continue
            if object_name == b'cf2' or object_name == b'base_link':  
                continue
            pos= p.getBasePositionAndOrientation(obstacle_id)[0]
            d = sqrt((aabb_max[0]-aabb_min[0])**2 + (aabb_max[1]-aabb_min[1])**2 + (aabb_max[2]-aabb_min[2])**2)/2
            velocity = p.getBaseVelocity(obstacle_id)[0]
            x_vel,y_vel,z_vel = velocity[0],velocity[1],velocity[2]            
            info = {"d_obs":d,"d_robot":2*(0.0397+2.31348e-2),"x_center":pos[0] ,"y_center":pos[1],"z_center":pos[2],"x_vel":x_vel,"y_vel":y_vel,"z_vel":z_vel }
            self.dynamic_objects[str(obstacle_id)] = info
    
    def goals_obstacles_info(self):
        goals_bodies = self.goals_obstacles 
        for obstacle_id in goals_bodies:
            aabb_min, aabb_max = p.getAABB(obstacle_id)
            object_name = p.getBodyInfo(obstacle_id)[0]  
            if object_name == b'plane' or object_name == b'planeLink':
                continue
            if object_name == b'cf2' or object_name == b'base_link':  
                continue
            pos_and_orientation= p.getBasePositionAndOrientation(obstacle_id)
            pos = pos_and_orientation[0]
            print("Info goals")
            d = sqrt((aabb_max[0]-aabb_min[0])**2 + (aabb_max[1]-aabb_min[1])**2 + (aabb_max[2]-aabb_min[2])**2)/2
            info = {"d_obs":d,"d_robot":2*(0.0397+2.31348e-2),"x_center":pos[0] ,"y_center":pos[1],"z_center":pos[2]}
            self.goals_objects.append(info)

    



