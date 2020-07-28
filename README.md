ROS Package is in the point_cloud_processing folder.

to clear the octomap error:
```
sudo apt-get install ros-kinetic-octomap ros-kinetic-octomap-mapping ros-kinetic-octomap-msgs ros-kinetic-octomap-ros ros-kinetic-octomap-rviz-plugins ros-kinetic-octomap-server
source ~/catkin_ws/devel/setup.zsh 

roscore
roslaunch point_cloud_processing transform.launch
cd /media/ksa/ubuntu_storage/works
rosbag play Gazebo_0.5m_Ground.bag -l
rviz rviz
```

Install the following packages to run farm_generation.py:
```
sudo apt-get install python3-sympy
# you can also install:
sudo apt-get install python3-numpy python3-scipy python3-matplotlib ipython ipython-notebook python3-pandas  python3-nose
#installing collada
sudo apt-get update -y
pip3 install pycollada
```

In order to run farm_generation.py, take care of the path in the following files:
1. models.txt
2. Empty world
3. soy.model


Datasets:   
Kentland Wheat Farm   
https://drive.google.com/open?id=11Gq6jv23fsi0puPGos4_p8Nh1qeF4EdX

Gazebo Soybean Farms:   
https://drive.google.com/open?id=1cbuy1wDpMB792Pbp7qokSVROeLPORPQV
