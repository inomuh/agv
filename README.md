# AGV-OTA (ROS Noetic)
This repository includes the AGV ROS Noetic packages."main" branch is latest version of AGV-OTA.

![Image of AGV](https://github.com/inomuh/agv/blob/main/images/agv_gazebo.png)

- agv_description: It is the sub-package containing urdf and mesh files of the AGV.
- agv_simulation: It is a sub-package containing the package and launch files required for the simulation of the AGV.
- agv_slam: It is a sub-package containing the slam_gmapping launch files.
- agv_navigation: It is a sub-package containing the navigation launch and config files.
- (NEW) agv_multirobot: It is a sub-package containing the AGV-OTA multirobot application files.


### For other AGV ROS Packages:
https://github.com/inomuh/agvpc_ros

https://github.com/inomuh/agvsim_v1_ros

https://github.com/inomuh/agvpc_ros2

https://github.com/inomuh/agvsim_v2_ros

Launch Command:
---------------
### Warning !!!
Before using launch commands, you must unzip ~/agv/agv_description/meshes/OTAv07_meshes/OTA-v0.7.tar.xz file..

-------------------------------------------------------------------------------------------------------------
Gazebo Launching:

    $ roslaunch agv_simulation agv_gazebo_emptyworld.launch

Gazebo (Depo Map) Launching:

    $ roslaunch agv_simulation agv_gazebo_depo.launch

Solo-Rviz Launching:

    $ roslaunch agv_simulation agv_rviz_standalone.launch
    
Rviz (with Gazebo) Launching:

    $ roslaunch agv_simulation agv_rviz.launch
    
SLAM Mapping Launching (must work with Gazebo and RViz Launching):

    $ roslaunch agv_slam agv_slam.launch
    
Navigation Launching:

    $ roslaunch agv_navigation agv_navigation.launch


![Image of AGV Navigation](https://github.com/inomuh/agv/blob/main/images/agv_nav_goal.png)
    
-----------------------------------------------------------------------------------------------------------------------
Requirements:
-------------
- In order for the "joint_state_publisher" to work, "joint_state_publisher_gui" package must be downloaded to your computer.

        $ sudo apt update && sudo apt install ros-noetic-joint-state-publisher-gui
        
- In order for the "joint_state_controller" to work, "joint_state_controller_gui" package must be downloaded to your computer.

        $ sudo apt install ros-noetic-ros-controllers
        
- In order for the SLAM to work, "slam_gmapping" package must be downloaded to your workspace.
        
        $ cd ~/catkin_ws/src && git clone https://github.com/ros-perception/slam_gmapping.git -b melodic-devel
        
- In order for the sensors to work properly, "gazebo_ros_pkgs" files must be downloaded to your computer.

        $ sudo apt-get install ros-noetic-gazebo-ros-pkgs
        
- In order for the navigation tools to work properly, "ros-navigation" files must be downloaded to your computer.

        $ sudo apt-get install ros-noetic-ros-navigation
        
- In order for the multirobot applications to work properly, "robot_state_publisher" package must be downloaded to your workspace.
        
        $ cd ~/catkin_ws/src && git clone https://github.com/rhaschke/robot_state_publisher -b noetic-devel
        
-------------------------------------------------------------------------------
Changelog:
----------
Update v1.0 - 07.12.20
----------------------
- First version

Update v1.1 - 07.01.21
----------------------
- Added AGV-OTA multirobot functionality with "agv_multirobot" subpackage.
- Some bug fixes.

- For Multirobot Launching:

        $ roslaunch agv_multirobot agv_multirobot.launch
        
![Image of AGV-Multirobot](https://github.com/inomuh/agv/blob/v1.1/images/agv_multirobot.png)

- For basic application of AGV-OTA Multirobots:

        $ rosrun agv_multirobot multi_move.py
        
Update v1.1.1 - 07.01.2021
--------------------------
- Bug fixes on "agv_multirobot" subpackage about "tf_prefix" and "robot_state_publisher"
- In order for the multirobot applications to work properly, "robot_state_publisher" package must be downloaded to your workspace.
        
        $ cd ~/catkin_ws/src && git clone https://github.com/rhaschke/robot_state_publisher -b noetic-devel

