# Zendar ROS Driver
Zendar ROS driver is an open source library for demonstrating how to convert Zendar radar data streams to ROS streams using ZendarAPI. Specifically, this library receives incoming radar data streams, converts them to ROS topics and publish them in real-time. This allows the radar data to be visualized using standard viewers such as [WebViz](https://webviz.io/) or [FoxGlove](https://foxglove.dev/).

## Setup Instructions
### Install Dependencies

#### Install ROS

If you do not already have ROS installed, please follow the instructions on the official ROS website at http://wiki.ros.org/ROS/Installation. Our software is currently supported on ROS version Noetic.

Afterward, run the following:
```
sudo apt install ros-noetic-perception
sudo apt install ros-noetic-rosbridge-suite 
```
Export the ROS paths by adding 
```
source /opt/ros/noetic/setup.bash
```
to your .bashrc script (typically in ~/.bashrc).

#### Install Other External Dependencies
Run the following command to install all other required dependencies: 
```
sudo apt install pcl protobuf-compiler libpcl-dev libgoogle-glog-dev ros-noetic-pcl-ros
```

#### Install Zendar Host Package
Install the host package supplied by Zendar with the following command:
```
sudo apt install ./zendar_host_latest.deb
```

### Setup zendar_ros_driver Repository
Clone the zendar_ros_driver repository:
```
git@github.com:ZendarInc/zendar_ros_driver.git
```
Source your .bash_profile:
```
source ~/.bash_profile
```
## Build Instructions
Go into the zendar_ros_driver repository:
```
cd /PATH_TO_ZENDAR_ROS_DRIVER/
```
Create, and step into build folder:
```
mkdir -p build && cd build
```
Build the driver:
```
cmake ..
make -j4
```
Source the driver by adding the following line to your .bashrc script (typically in ~/.bashrc):
```
source /PATH_TO_ZENDAR_ROS_DRIVER/build/devel/setup.bash
```

## Running Zendar ROS Driver
### Real-time Streaming
After the radars are started, the ROS plugin for visualization can be run:
```
roslaunch zendar_ros_driver zendar.launch target_url:=URL
```
where URL needs to be replaced with the url of the ZPU.

### Recording ROS Bags
It is better to record only the necessary topics due to size and bandwidth limitations. 
You can use the prepackaged launch file that records all necessary topics by running the following command anywhere in the terminal.
```
roslaunch zendar_ros_driver rosbag_save.launch output_name:=/home/zendar_data/record.bag
```
The default storage location is the user’s home directory, and the date and time of the recording will be appended to the file name.

## Useful ROS Commands

| Command                                                                |                                                                                       Description                                                                                        |
|------------------------------------------------------------------------|:----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------:|
| `roscore`                                                              |                                                            start a rosmaster, parameter server and <br/> rosout logging node                                                             |
| `rosbag record -a`                                                     |                                                                       records data from all topics (-a flag = all)                                                                       |
| `rosnode list`                                                         |                                                                                 lists all current nodes                                                                                  |
| `rosnode info <NODE_NAME>`                                             |                                                                              print information about nodes                                                                               |
| `rostopic hz <TOPIC_NAME>`                                             |                                                                             display publishing rate of topic                                                                             |
| `rosnode ping <NODE_NAME>`                                             |                                                                                test connectivity to node                                                                                 |
| `rostopic list <TOPIC_NAME>`                                           |                                                                          print information about active topics                                                                           |
| `rostopic type <TOPIC_NAME>`                                           |                                                                                     print topic type                                                                                     |
| `rostopic info <TOPIC_NAME>`                                           |                                                                           print information about active topic                                                                           |
| `rostopic echo <TOPIC_NAME>`                                           |                                                       print messages to screen, useful to check if topics are actually publishing                                                        |
| <code> rostopic echo <TOPIC_NAME> &vert; grep -B 1 -A 1 keyword</code> | print part of message containing `keyword` to screen where `-B` defines the number of lines before the keyword that are printed, and  `-A` defines the number of lines after the keyword |
| `rostopic bw <TOPIC_NAME>`                                             |                                                                             display bandwidth used by topic                                                                              |
| `rostopic delay <TOPIC_NAME>`                                          |                                                                                 display delay for topic                                                                                  |
| `rosrun tf tf_echo /parent /child`                                     |                                                                          display transform from child to parent                                                                          |