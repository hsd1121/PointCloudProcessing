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

Datasets:

Kentland Wheat Farm

https://drive.google.com/open?id=11Gq6jv23fsi0puPGos4_p8Nh1qeF4EdX

Gazebo Soybean Farms

https://drive.google.com/open?id=1cbuy1wDpMB792Pbp7qokSVROeLPORPQV
