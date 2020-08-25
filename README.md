# IHA-4506 Advanced Robotics 2020
We'll use [[4]](https://github.com/modulabs/arm-control) for implementing and demonstrating the course assignments.  

## Prerequisit
It is recommended to use the following requirements to avoid any unforseen error during the implementation of the codes.

- Ubuntu 18
- ROS Melodic

## VMWare (Optional)
If you are using windows or run an older version of ubuntu on your PC, you can use VMWare Workstation Player to run ubuntu 18 which is free for non-commercial use. In that case, follow the instructions below to install and prepare you virtual machine:

- depending on your current version of operating system, download one of the installers from [VMWare Installers](https://www.vmware.com/fi/products/workstation-player/workstation-player-evaluation.html).
- on windows, just run the installer GUI and follow the instructions
- on Ubuntu install it from command lines:
```
$ cd "PATH/TO/DOWNLOADED/FOLDER"  #change directory to where you have downloaded the .bundle (installer) file
$ sudo sh VMware-Player-15.5.6-16341506.x86_64.bundle  #change .bundle file name if necessary
```
- Download an ubuntu .iso image from [Ubuntu 18 ISO file](https://releases.ubuntu.com/bionic/ubuntu-18.04.5-desktop-amd64.iso)
- open VMWare Workstation Player and create an new environment from the ISO file and set the hardware properties as you wish.

## ROS melodic
To avoid any possible incompatibility issue, please install ROS melodic from [ROS Melodic installation guide](http://wiki.ros.org/melodic/Installation/Ubuntu)

## installation
The github repository provided here is the modified version of [4] that some of its bugs are fixed and we'll maintain it during the course implementation. 

### Download and build 

    $ mkdir -p catkin_ws/src
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/AdvancedRobotics-iha4506/ElfinSimulation.git
    $ cd ~/catkin_ws/
    $ catkin_make

### Run examples
Motion controllers in joint space

    $ roslaunch elfin_gazebo elfin3_empty_world.launch controller:=gravity_comp_controller
    or
    $ roslaunch elfin_gazebo elfin3_empty_world.launch controller:=time_delay_controller
    or
    $ roslaunch elfin_gazebo elfin3_empty_world.launch controller:=computed_torque_controller
    or
    $ roslaunch elfin_gazebo elfin3_empty_world.launch controller:=computed_torque_clik_controller

If you want to use motion and force controller in task space, then you may choose this controllers as follows:

    $ roslaunch elfin_gazebo elfin3_experiment1_world.launch controller:=adaptive_impedance_controller
    or
    $ roslaunch elfin_gazebo elfin3_experiment2_world.launch controller:=adaptive_impedance_controller

If you want to plot data in rqt graph, use rqt_plot.launch file. Customize perspective files to plot data you need.

    $ roslaunch rqt_plot.launch controller:=gravity_comp_controller



## Notes
- If you are using VMWare and gazebo keeps crashing during run time, try the following:

```
$ echo "export SVGA_VGPU10=0" >> ~/.bashrc
``` 
- before running any of the sample codes, you also need to issue the following command from your /catkin_ws directory so the built nodes and launch files could be found in terminal:


```
$ source devel/setup.bash
```
## References
1. [ros_Control](http://wiki.ros.org/ros_control)
2. [Write a new ros-controller](https://github.com/ros-controls/ros_control/wiki/controller_interface)
3. [Elfin Robot](http://wiki.ros.org/Robots/Elfin)
4. [Elfin Simulation package](https://github.com/modulabs/arm-control)
5. [ROS](http://wiki.ros.org/)

