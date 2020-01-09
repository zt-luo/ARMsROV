# ARMsROV simulator  

base on [uuv_simulator](https://github.com/uuvsimulator/uuv_simulator).

## Description

### armsrov_description  

ARMsROV description include mesh file and model.  

### armsrov_bridge  

a bridge to [SubControl](https://github.com/zt-luo/SubControl).  

## build

**requirements: Ubuntu 18.04, ROS melodic.**

### install uuv_simulator

``` bash
sudo apt install ros-melodic-uuv-simulator
```

### build

``` bash
mkdir ARMsROV
# clone repo
git clone https://github.com/zt-luo/ARMsROV.git src --recursive

# build
catkin_make
```

## quick start

``` bash
# new terminal
# launch gazebo world
roslaunch uuv_gazebo_worlds ocean_waves.launch
```

``` bash
# new terminal
# launch armsrov_description
source devel/setup.bash
roslaunch armsrov_description upload.launch
```

``` bash
# new terminal
# run armsrov_bridge
source devel/setup.bash
$ rosrun armsrov_bridge armsrov_bridge_node          
```
