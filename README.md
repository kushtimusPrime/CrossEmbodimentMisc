# Cross Embodiment

Hi. Welcome to this test workspace for the cross embodiment project. This is currently in development.

## Repo Setup
```
mkdir -p ~/cross_embodiment_test_ws/src
cd ~/cross_embodiment_test_ws
git clone <LINK> src
colcon build
. install/setup.bash
```

## Fk_Ik_Rviz_Pkg
This package has python scripts to analyze URDFs to provide forward and inverse kinematics.

## Gazebo_Env
This package will launch a UR5E robot in Gazebo. It has simulated hardware control so you can move the robot to the desired position. A sample script is also provided to move the robot to some preplanned positions.

Need to import kinpy
