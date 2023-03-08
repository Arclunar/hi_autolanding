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
    //用于获得真实坐标
    ros::ServiceClient client=n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    int seq=0;//发布序号
    while(ros::ok())
    {
        gazebo_msgs::GetModelState srv;
        srv.request.model_name = robot_name; //指定要获取的机器人在gazebo中的名字；
        if (client.call(srv))
        {
            nav_msgs::Odometry odom;
            odom.header.frame_id="odom";
            odom.header.seq=seq++;
            odom.header.stamp=ros::Time::now();
            odom.pose.pose.position=srv.response.pose.position;
            odom.pose.pose.orientation=srv.response.pose.orientation;
            odom.twist.twist.linear=srv.response.twist.linear;
            odom.twist.twist.angular=srv.response.twist.angular;
            odom_publisher.publish(odom);

            // geometry_msgs::PoseStamped pose;
            // pose.header.frame_id="odom";
            // pose.header.seq=seq++;
            // pose.header.stamp=ros::Time::now();
            // pose.pose=srv.response.pose;
            // pose_publisher.publish(pose);
        }
        else
        {
            ROS_ERROR("Failed to call service /gazebo/get_model_state");
        }
        loop_rate.sleep();
    }

    return 0;
}
