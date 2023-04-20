/*
 * @Author: Jimazeyu
 * @Date: 2022-02-18 03:19:20
 * @LastEditors: OBKoro1
 * @LastEditTime: 2022-02-18 04:38:48
 * @FilePath: /catkin_ws/src/map_tutorials/src/pose_publisher.cpp
 */
#include"ros/ros.h"
#include"iostream"
#include"gazebo_msgs/GetModelState.h"
#include"geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
using namespace std;

nav_msgs::Odometry diff_odom;
nav_msgs::Odometry drone_odom;
nav_msgs::Odometry car_odom;


//*********************
//小车base和apriltag的相对位置
// At time 0.000
// - Translation: [0.081, 0.000, 0.265]
// - Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]
//             in RPY (radian) [0.000, -0.000, 0.000]
//             in RPY (degree) [0.000, -0.000, 0.000]




int main(int argc,char**argv)
{
    ros::init(argc, argv, "true_pose_publisher");
    ros::NodeHandle n("~");
    
    int pub_rate=100;
    n.getParam("/vins_fusion/true_odom_pub_rate", pub_rate);
    ros::Rate loop_rate(pub_rate);

    //获取机器人名字
    string robot_name;
    n.param<std::string>("robot_name", robot_name, "iris");
    cout<<"true odom publish : "<<robot_name<<endl;
    //用于发布机器人真实坐标
    // ros::Publisher pose_publisher=n.advertise<geometry_msgs::PoseStamped>("true_pose",10);
    ros::Publisher odom_publisher=n.advertise<nav_msgs::Odometry>("/vins_fusion/imu_propagate",10);
    ros::Publisher car_odom_publisher = n.advertise<nav_msgs::Odometry>("husky_odom",10);
    ros::Publisher apritag_odom_publisher = n.advertise<nav_msgs::Odometry>("apriltag_odom",10);
    ros::Publisher diff_odom_publisher = n.advertise<nav_msgs::Odometry>("/diff_odom",10);



    //用于获得真实坐标
    ros::ServiceClient client=n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    int drone_seq=0;//发布序号
    int car_seq=0;//发布序号
    int diff_seq=0;
    while(ros::ok())
    {
        gazebo_msgs::GetModelState srv;
        srv.request.model_name = robot_name; //指定要获取的机器人在gazebo中的名字；
        if (client.call(srv))
        {
            drone_odom.header.frame_id="world";
            drone_odom.header.seq=drone_seq++;
            drone_odom.header.stamp=ros::Time::now();
            drone_odom.pose.pose.position=srv.response.pose.position;
            drone_odom.pose.pose.orientation=srv.response.pose.orientation;
            drone_odom.twist.twist.linear=srv.response.twist.linear;
            drone_odom.twist.twist.angular=srv.response.twist.angular;
            odom_publisher.publish(drone_odom);

            // geometry_msgs::PoseStamped pose;
            // pose.header.frame_id="odom";
            // pose.header.seq=seq++;
            // pose.header.stamp=ros::Time::now();
            // pose.pose=srv.response.pose;
            // pose_publisher.publish(pose);
        }
        else
        {
            ROS_ERROR("Failed to call service /gazebo/get_model_state for iris");
        }
        srv.request.model_name="husky";
        if(client.call(srv))
        {
            car_odom.header.frame_id="world";
            car_odom.header.seq=car_seq++;
            car_odom.header.stamp=ros::Time::now();
            car_odom.pose.pose.position=srv.response.pose.position;
            car_odom.pose.pose.orientation=srv.response.pose.orientation;
            car_odom.twist.twist.linear=srv.response.twist.linear;
            car_odom.twist.twist.angular=srv.response.twist.angular;
            car_odom_publisher.publish(car_odom);
            car_odom.pose.pose.position.x=srv.response.pose.position.x+0.081;
            car_odom.pose.pose.position.z=srv.response.pose.position.z+0.265;
            apritag_odom_publisher.publish(car_odom);
        }
        else{
           ROS_ERROR("Failed to call service /gazebo/get_model_state for husky");

        }

        diff_odom.header.frame_id="world";
        diff_odom.header.seq=diff_seq++;
        diff_odom.header.stamp=ros::Time::now();
        diff_odom.pose.pose.position.x= car_odom.pose.pose.position.x - drone_odom.pose.pose.position.x;
        diff_odom.pose.pose.position.y= car_odom.pose.pose.position.y - drone_odom.pose.pose.position.y;
        diff_odom.pose.pose.position.z= car_odom.pose.pose.position.z - drone_odom.pose.pose.position.z;
        diff_odom_publisher.publish(diff_odom);
        loop_rate.sleep();
    }

    return 0;
}
