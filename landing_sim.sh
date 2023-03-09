# 不用管端口
# sudo cpufreq-set -g performance;
# sudo chmod 777 /dev/px4;#飞控/dev/ttyACM0
# sudo chmod 777 /dev/ttyUSB_laser;#测距/dev/ttyUSB_laser
# sudo chmod 777 /dev/ttyUSB_relay;#继电器


# sudo udevadm trigger & sleep 1; # 不知道干嘛的
roslaunch sim_node simauto_landing.launch & sleep 1; 
# 启动仿真环境和里程计发布节点，px4固件
 

# source devel/setup.bash;
# roslaunch uavsensor rccmd.launch & sleep 1;
roslaunch sim_node rccmd.launch & sleep 1;

# roslaunch uavsensor relay.launch & sleep 0.5;   # 继电器

#roslaunch uavsensor laser.launch & sleep 0.5;
# roslaunch nlink_parser tofsense.launch & sleep 0.5; # gazebo直接获取激光信息



# roslaunch realsense2_camera rs_camera.launch & sleep 4;

# roslaunch mavros px4.launch & sleep 3; # 启用mavros

# 修改mavlink接收imu频率
rosrun mavros mavcmd long 511 105 4000 0 0 0 0 0 & sleep 1;
rosrun mavros mavcmd long 511 31 4000 0 0 0 0 0 & sleep 1; 

# roslaunch vins vins_fast330_v5_rs435.launch & sleep 6; # 不开vins

roslaunch px4ctrl run_ctrl.launch & sleep 1;

# roslaunch usb_cam run_double_cam.launch & sleep 3;

roslaunch mapping run.launch & sleep 1; # 测试完成，没有问题
roslaunch planning run.launch & sleep 1; # 启动 planning nodelet 以及 traj server

roslaunch mapping rviz_real.launch & sleep 1 ; # 启动rviz结点

# 启动ekf
roslaunch target_ekf target_ekf.launch  & sleep 1; 

# apriltag 检测
source /home/anifan/apriltag_ws/devel_isolated/setup.bash; 
roslaunch apriltag_ros double_detection_sim.launch;
# roslaunch apriltag_ros continuous_detection.launch;

wait;
