# rosbag record --tcpnodelay /vins_estimator/imu_propagate /position_cmd /drone0/gridmap_inflate /drone0/replanState /target_odom /trtpose/body_kp /drone0/heartbeat /drone0/planning/polyhedra /drone0/trajectory /drone0/planning/traj /drone0/planning/astar 
rosbag record --tcpnodelay /target_ekf_node/target_odom /position_cmd /drone0/gridmap_inflate /drone0/planning/polyhedra /drone0/trajectory /drone0/planning/traj /drone0/planning/astar /tag_detections /vins_fusion/imu_propagate /land_triger  /relayctrl_node/state /relayctrl_node/cmd /april_tag_down/tag_detections /april_tag_down/tag_detections_image /april_tag_front/tag_detections_image /tfluna_node/data

