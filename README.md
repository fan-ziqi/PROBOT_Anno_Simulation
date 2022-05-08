# PROBOT Anno 机械臂仿真

## 说明

本仿真实现了机械臂使用视觉识别并抓取不同颜色的物块

## 环境安装

1. 安装完整的ROS（Noetic版本）
2. 安装gazebo11
    ```sh
    sudo apt install ros-noetic-gazebo-*
    ```
3. 安装OpenCV
    ```
    sudo apt install libopencv-dev python3-opencv
    ```
4. 安装其他依赖 
    ```sh
    ROS_VERSION=`/usr/bin/rosversion -d`   
    sudo apt-get install ros-${ROS_VERSION}-moveit-*   
    sudo apt-get install ros-${ROS_VERSION}-industrial-*   
    sudo apt-get install ros-${ROS_VERSION}-gazebo-ros-control   
    sudo apt-get install ros-${ROS_VERSION}-ros-control ros-${ROS_VERSION}-ros-controllers   
    sudo apt-get install ros-${ROS_VERSION}-trac-ik-kinematics-plugin   
    sudo apt-get install ros-${ROS_VERSION}-usb-cam   
    ```
## 编译使用
- 创建功能包
- 使用 "catkin_make" 编译
- 设置环境变量:   
  ```sh
  $ echo "source ~/'WORKSPACE_PATH'/install/setup.bash" >> ~/.bashrc
  $ source ~/.bashrc
  ```

## 运行仿真 

```sh
roslaunch probot_grasping probot_anno_grasping_demo.launch
```
