roslaunch mapping run.launch & sleep 1; # 测试完成，没有问题
roslaunch planning run.launch & sleep 1; # 启动 planning nodelet 以及 traj server

roslaunch mapping rviz_real.launch & sleep 1 ; # 启动rviz结点

wait;