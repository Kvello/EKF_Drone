# Simplified Extended Kalman Filter on a Drone

## :memo: About

This project is part of the EE4308 - Autonomous Robot Systemes course taken at NUS spring 2024.
The project description can be found in the pdf in this repo.

## :gear: Setup

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

## :zap: Running

To run the project you can simply use

```{bash}
. run.sh
```

## Non-trivial implementation details and justifications:

The system has been extened with a statistics package that calculates the covariances for the measurements. See the relevant readme for more.
Since the measurements calculate the variance of the x,y vector of the magnetometer measurement, it is necessary to transform the measurements. This means that the variance of the angle measuremements is assumed to depend on the position of the drone. This is more likely due to the highly non-linear transform.
