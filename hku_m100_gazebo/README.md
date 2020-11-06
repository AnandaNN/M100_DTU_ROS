# M100 Gazebo package

Credits: caochao39 [hku_m100_gazebo](https://github.com/caochao39/hku_m100_gazebo)

A ROS package for simulating the DJI Matrice 100 Drone in Gazebo

**Usage**
Open the gazebo simulation
> $ roslaunch hku_m100_gazebo wall_simulation.launch

You should see in the gazebo world a parking Matrice 100 and a concrete wall

Connect the drone in Gazebo with the drone in the PC simulator
> $ rosrun hku_m100_gazebo m100_sim_bridge

Control the drone using the DJI remote controller, you should see the drone flying in the gazebo world
