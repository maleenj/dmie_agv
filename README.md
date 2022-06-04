# DMIE AGV

See [DMIE_AGV Wiki](https://github.com/maleenj/dmie_agv/wiki) for more details

## INSTALLION PREREQUISITES

Ubuntu 20.04

ROS Noetic

### Mapping

ROS Gmapping Package: http://wiki.ros.org/gmapping

```
sudo apt-get install ros-noetic-gmapping
```

### Localisation

ROS AMCL Package: http://wiki.ros.org/amcl

```
sudo apt-get install ros-noetic-amcl
```

### Motion Planning

ROS Move Base Package: http://wiki.ros.org/move_base

```
sudo apt-get install ros-noetic-move-base
```
### Simulation

CAn use any Gazebo simulation environment. Example:

Use AWS RoboMaker Small Warehouse World

https://github.com/aws-robotics/aws-robomaker-small-warehouse-world

```
cd ~/catkin_ws/src/
git clone https://github.com/aws-robotics/aws-robomaker-small-warehouse-world.git
cd ..
catkin_make
```
## INSTALL DMIE_AGV PACKAGE

```
cd ~/catkin_ws/src/
git clone https://github.com/maleenj/dmie_agv.git
cd ..
catkin_make
```
