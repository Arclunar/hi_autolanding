# 仿真

## 环境配置&#x20;

按照fast drone250配置好

-   Husky\_ws

    参考文章&#x20;

    [(125条消息) ROS仿真环境配置Husky+32线激光雷达+深度相机+imu+gps\_zhhao1326的博客-CSDN博客](https://blog.csdn.net/weixin_42438635/article/details/125304796 "(125条消息) ROS仿真环境配置Husky+32线激光雷达+深度相机+imu+gps_zhhao1326的博客-CSDN博客")

    会报错，所以要依赖

    依赖

    参考文章&#x20;

    [How to use ros-noetic-husky? - ROS Answers: Open Source Q\&A Forum](https://answers.ros.org/question/385568/how-to-use-ros-noetic-husky/ "How to use ros-noetic-husky? - ROS Answers: Open Source Q\&A Forum")

    安装
    ```bash
    mkdir Husky_ws/src
    cd Husky_ws/src

    #下载指定分支HUSKY
    git clone -b noetic-devel https://github.com/husky/husky.git

    cd ..
    catkin_make
    ```
    ```bash
    sudo apt-get install ros-noetic-lms1xx 
    ```
    ```bash
    sudo apt-get install ros-noetic-robot-localization 
    ```
    ```bash
    sudo apt-get install ros-noetic-interactive-marker-twist-server 
    ```
    ```bash
    sudo apt-get install ros-noetic-joy
    ```
    ```bash
    sudo apt-get install ros-noetic-twist-mux  
    ```
    ```bash
    sudo apt-get install ros-noetic-teleop-twist-joy
    ```
    ```bash
    sudo apt-get install ros-noetic-dwa-local-planner 
    ```
    ```bash
    sudo apt-get install ros-noetic-velodyne-description 
    ```
    还是不能启动，把husky模型换成我的husky模型

    在xacro那里

    gazebo apriltag模型

    纹理添加在

    /usr/share/gazebo-11/media/materials/textures 和 .../materials/scripts

    启动
    ```bash
    roslaunch husky_gazebo husky_empty_world.launch
    # roslaunch husky_gazebo husky_playpen.launch
    #自定义的环境
    # roslaunch husky_gazebo husky_empty_world.launch world_name:=worlds/willowgarage.world

    rosrun turtlesim turtle_teleop_key turtle1/cmd_vel:=husky/cmd_vel

    ```
-   PX4-Autopilot
    -   安装
        -   参考
            [(125条消息) Ubuntu20.04+MAVROS+PX4+Gazebo保姆级安装教程\_晨少的博客的博客-CSDN博客](https://blog.csdn.net/HuangChen666/article/details/128754106 "(125条消息) Ubuntu20.04+MAVROS+PX4+Gazebo保姆级安装教程_晨少的博客的博客-CSDN博客")
        ```bash
        pip install —upgrade  numpy
        ```
        ```bash
        git clone https://github.com/PX4/PX4-Autopilot.git --recursive
        ```
        ```bash
        cd PX4-Autopilot/
        git submodule update --init --recursive

        ```
        ```bash
        cd .. & bash ./PX4-Autopilot/Tools/setup/ubuntu.sh  

        ```
        重启
        ```bash
        make px4_sitl_default gazebo

        ```
        添加环境变量
        ```bash
        source ~/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default
        export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot
        export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic

        ```

realsense参考文章

[https://blog.csdn.net/sinat\_16643223/article/details/120072154](https://blog.csdn.net/sinat_16643223/article/details/120072154 "https://blog.csdn.net/sinat_16643223/article/details/120072154")&#x20;

距离传感器

用的是 单线的激光，使用lidar模型，但是插件改成laser的插件，参考问文章

[https://discuss.px4.io/t/add-distance-sensor-to-drone-model-in-gazebo/28936](https://discuss.px4.io/t/add-distance-sensor-to-drone-model-in-gazebo/28936 "https://discuss.px4.io/t/add-distance-sensor-to-drone-model-in-gazebo/28936")

sdf 格式

参考文章

[https://zhuanlan.zhihu.com/p/129659674](https://zhuanlan.zhihu.com/p/129659674 "https://zhuanlan.zhihu.com/p/129659674")

## 编译&#x20;

#### 补充库

serial库

sudo apt install ros-noetic-serial

发现找不到lserial库：

get the code:

git clone [https://github.com/nooploop-dev/serial.git](https://github.com/nooploop-dev/serial.git "https://github.com/nooploop-dev/serial.git")

Build:

make

Build and run the tests:

make test

Install:

sudo make install

#### 源文件编译顺序&#x20;

src

先编译 quadrotor\_msgs

detection

然后 uav\_simulators

然后 realflightmodule

这里先编译n，最后再编译px4ctrl

最后planning

## 运行

控制飞机的遥控器功能键确实是通道9，在uavsensor功能包里面的rccmd\_node里

打开仿真遥控结点

```bash
roslaunch sim_node rccmd.launch
```

先运行target\_ekf.launch 否则该节点会打不开（或者在target\_ekf.cpp里面删掉第一句话和时间有关）

```bash
roslaunch target_ekf target_ekf.launch 
```

打开仿真环境

```bash
roslaunch sim_node simauto_landing.launch
```

更改imu接收频率

```bash
rosrun mavros mavcmd long 511 105 4000 0 0 0 0 0 & sleep 1;
```

```bash
rosrun mavros mavcmd long 511 31 4000 0 0 0 0 0 & sleep 1;
```

打开apriltag检测

```bash
source ~/apriltag_ws/devel_isolated/setup.bash; 
roslaunch apriltag_ros sim_double_detection.launch;
```

打开控制、建图、导航模块

```bash
roslaunch px4ctrl run_ctrl.launch & sleep 1;
roslaunch mapping run.launch & sleep 1; 
roslaunch planning run.launch & sleep 1; 
```

Husky控制 向前1m/s速度前进

```bash
rostopic pub -r 10 /husky_velocity_controller/cmd_vel geometry_msgs/Twist "linear:
  x: 0.1
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

起飞

```bash
./takeoff.sh
```

跟踪

```bash
./track.sh
```

降落

```bash
./land.sh
```

就地降落

```bash
./down.sh 
```
