# Quadrotor3D

# Description of this README file
This README file explains the content of the planning and decision making project. First the assignment will be explained together with the goal.  Afterwards the packages required to setup and run the environment will be reviewed. 
# 

# Description of the task
The goal of this project was to make a path planning algorithm which is able to dynamically avoid objects during its route. Furthermore the algorithm is intended for a quadrotor. Finally for the environment the Quadrotor must avoid objects like cube and must go through a torus without hitting it.

For the implementation of MPC to the quadrotor both safe-control-gym was used, which uses the pybullet environment as a skeleton.

The safe-control-gym and pybullet packages already provide the packages and objects necessairy to setup the environment. To implement the MPC-controller properly the trajectory and the algorithm must be further developed. 

# 
# Description of the packages, nodes and launch files
This project package houses the functions necessairy to generate an environment with the desired object, create a path and trajectory, quadrotor and its dynamics with constraints and finally the controller. The file structure of the packages can be seen below. By running this packages together with the provided simulator, the Quadrotor should be able to drive autonomously through the path, without hitting an object.
```
.
├── 3d_quad.yaml
├── assets
│   ├── cf2.dae
│   ├── cf2x.urdf
│   ├── gate.obj
│   ├── gate.urdf
│   ├── hb.urdf
│   ├── quad.obj
│   ├── ring.stl
│   ├── sphere.urdf
│   └── torus.urdf
├── base_aviary2.py
├── benchmark_env2.py
├── create_obstacles.py
├── MPC.py
├── Quad_env.py
├── Quadrotor_mpc.py
├── Quadrotor.py
├── Quadrotor_utils.py
├── results_dict_con3_more_obs2.pkl
├── results.py
├── Trajec_gen.py
└── videomaking.py
```
# 
# Instruction to build the solution  

Follow the commands below to build the solution, without this installation process the solution will not work. 
## 1. Installing anaconda:

    mkdir anaconda
    cd anaconda
    wget https://repo.anaconda.com/archive/Anaconda3-2022.05-Linux-x86_64.sh
    bash Anaconda3-2022.05-Linux-x86_64.sh
    cd ..
    
    
Complete the installation. After installation run this command:

    source ~/.bashrc

## 2. Cloning the assignment repo:

First clone the Quadrotor-MPC repo:
    
    git clone git@github.com:PDM-group07/Quadrotor-MPC.git
    cd Quadrotor-MPC
    
After cloning the Quadrotor-MPC repo, clone the safe-control-gym repo inside the Quadrotor3D folder:

    git clone https://github.com/utiasDSL/safe-control-gym.git
    cd safe-control-gym

Run the following commands:
    
    sudo apt-get install libgmp-dev
    conda create -n safe python=3.8.10
    conda activate safe
    pip install --upgrade pip
    pip install -e .
    pip install imageio_ffmpeg

To prevent errors with running inside the safe environment, execute the following commands:
    
    cd /home/$USER/anaconda3/envs/safe/lib
    mkdir backup
    mv libstd* backup 
    cp /usr/lib/x86_64-linux-gnu/libstdc++.so.6  ./
    ln -s libstdc++.so.6 libstdc++.so
    ln -s libstdc++.so.6 libstdc++.so.6.0.19

Close the terminal or exit the anaconda3 directory.

In order to run the simulation the following must be changed first:
    in safe-control-gym --> safe_control_gym --> utils --> registration.py:
    copy the following command in line 72:

        if self.entry_point == "safe_control_gym.envs.gym_pybullet_drones.quadrotor:Quadrotor":
            self.entry_point = "Quadrotor:Quadrotor"
            print(self.entry_point)

## 3. Instruction to run the solution
First make sure that you are in the right directory and also make sure that the safe environment is activated:

	conda activate safe
	cd ~/Quadrotor-MPC/Quadrotor

After cloning and installing, run the following command to launch the simulator:

	python Quadrotor_mpc.py --overrides ./3d_quad.yaml

If everything goes well, the environment will be launched and the simulation should run. If you run into problems with modules error, try to pip install the missing modules. 

