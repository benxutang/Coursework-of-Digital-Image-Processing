/*****************
 *  node name: controler
 *  subscribe topic name: 1. "/machinevision"：机器学习
 *                       2. "/trafficLaneTrack"：传统方法
 *  publish topic name:  "/smoother_cmd_vel"
 *  The function of the node:
 *    接受两种处理方法处理过的小车控制命令，通过逻辑语句，决定
 *  向控制小车的节点发布相关的控制信息。
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"

class SpeedControl{
public:
  ros::NodeHandle n;
  ros::Publisher speed_pub;
  ros::Subscriber speed_sub1;
  ros::Subscriber speed_sub2;
  int flag = 1;
  int count = 0;
  SpeedControl(){
    ros::Rate loop_rate(100);
    speed_pub = n.advertise<geometry_msgs::Twist>("/smoother_cmd_vel", 5);
    speed_sub1 = n.subscribe("/machinespeed", 1000, &SpeedControl::messagecallback1, this);
    speed_sub2 = n.subscribe("/hspeed", 1000, &SpeedControl::messagecallback2, this);
    ros::spin();
  }
  void messagecallback1(const geometry_msgs::Twist &msg1){
    geometry_msgs::Twist speed1;
    if (1 == msg1.linear.y && 1 == flag){
      speed1.linear.x = msg1.linear.x;
      speed1.angular.z = msg1.angular.z;
      speed_pub.publish(speed1);
    }
    else  count++;
    if (count > 50) flag = 0;
  }
  void messagecallback2(const geometry_msgs::Twist &msg2){
    geometry_msgs::Twist speed2;
    speed2.linear.x = msg2.linear.x;
    speed2.angular.z = msg2.angular.z;
    speed_pub.publish(speed2);
  }
};

int main(int argc, char **argv){
  ROS_WARN("*****START*****");
  ros::init(argc, argv, "controler");
  SpeedControl control;
  return 0;
}