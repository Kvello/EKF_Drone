# Simplified Extended Kalman Filter on a Drone
## :memo: About
This project is part of the EE4308 - Autonomous Robot Systemes course taken at NUS spring 2024.
The project description can be found in the pdf in this repo.
## :file_folder: Setup
To setup the project locally, start by cloning the repo:
```{bash}
git pull git@github.com:Kvello/EKF_Drone.git
```
Next, initialize the submodules with
```{bash}
git submodule init
git submodule update
```
alternatively you can run
```{bash}
git clone --recurse-submodules git@github.com:Kvello/EKF_Drone.git
```
to automatically initialize submodules when cloning
Next, build the project with
```{bash}
colcon build --symlink-install
```
