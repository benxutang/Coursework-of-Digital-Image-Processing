#include <ros/ros.h>
#include <stdlib.h>
#include <geometry_msgs/Twist.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/thread/thread.hpp>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Image.h"
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <fstream>
#include <iostream>

using namespace message_filters;

// typedef struct PID_Control{
//     float Kp;         // 比例参数
//     float Ki;         // 积分参数
//     float Kd;         // 微分参数
//     float err[3];
//     float target;     // 目标量
//     float fdb;        // 实时量
//     float output;     // 输出量
//     float outputMax;  // 输出限幅
// }PID;

class SubscribeAndPublish{
public:
  ros::NodeHandle n;        // 设置ROS句柄
  ros::Publisher speed_pub;
  SubscribeAndPublish(){
    ros::Rate loop_rate(100); // 定义速度发布频率
    speed_pub = n.advertise<geometry_msgs::Twist>("/mechinespeed", 5); // 定义速度发布器
    // speed_pub = n.advertise<geometry_msgs::Twist>("/smoother_cmd_vel", 5); // 定义速度发布器
    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> object_sub(n, "/darknet_ros/bounding_boxes", 1); // 定义方框信息接收器
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(n, "/zed/zed_node/depth/depth_registered", 1);      // 定义ZED深度摄像头信息接收器
    // 同步机器视觉和ZED深度摄像头两个话题的消息
    typedef sync_policies::ApproximateTime<darknet_ros_msgs::BoundingBoxes, sensor_msgs::Image> syncPolicy;
    Synchronizer<syncPolicy> sync(syncPolicy(10), object_sub, depth_sub);
    sync.registerCallback(boost::bind(&SubscribeAndPublish::depth_callback, this, _1, _2));
    ros::spin();
  }
  void depth_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &detect_msg, const sensor_msgs::Image::ConstPtr &msg)  {
    int x, y; // 定义框图中心
    int centerIdx;
    int sizes = msg->data.size();
    float *depths = (float *)(&msg->data[0]); // 定义照片深度信息
    int num = detect_msg->bounding_boxes.size();
    for (int i = 0; i < num; i++){
      x = ((detect_msg->bounding_boxes[i].xmax - detect_msg->bounding_boxes[i].xmin) / 2) + detect_msg->bounding_boxes[i].xmin;
      y = ((detect_msg->bounding_boxes[i].ymax - detect_msg->bounding_boxes[i].ymin) / 2) + detect_msg->bounding_boxes[i].ymin;
      centerIdx = x + msg->width * y;
      if (centerIdx < 0){
        centerIdx = 0;
      }
      else if (centerIdx > sizes/4){
        centerIdx = sizes / 4;
      }
    }
    geometry_msgs::Twist cmd_red;
    cmd_red.linear.y = 1;
    // 演示DEMO
    if (x <= 560)
      cmd_red.angular.z = 0.1;
    else if (x > 700)
      cmd_red.angular.z = -0.1;
    else
      cmd_red.angular.z = 0;

    if (depths[centerIdx] <= 0.5)
      cmd_red.linear.x = -0.1;
    else if (depths[centerIdx] > 1)
      cmd_red.linear.x = 0.1;
    else
      cmd_red.linear.x = 0;

    if(0==cmd_red.angular.z && 0== cmd_red.linear.x)
      cmd_red.linear.y = 0;

    speed_pub.publish(cmd_red);

    // // 左右运动的PID控制
    // PID pid_x;
    // pid_x.target = 640;
    // pid_x.fdb = x;
    // pid_x.outputMax = 1;
    // pid_x.Kp = 1;
    // pid_x.Ki = 0;
    // pid_x.Kd = 0;
    // pid_x.err[0] = pid_x.target - pid_x.fdb;
    // pid_x.output = pid_x.Kp*(pid_x.err[0]-pid_x.err[1])+pid_x.Ki*pid_x.err[0]+pid_x.Kd*(pid_x.err[0]+pid_x.err[2]-2*pid_x.err[1]);
    // pid_x.err[2] = pid_x.err[1];
    // pid_x.err[1] = pid_x.err[0];
    // if(pid_x.output > pid_x.outputMax) pid_x.output = pid_x.outputMax;
    // if(pid_x.output < -pid_x.outputMax) pid_x.output = -pid_x.outputMax;
    // // 前后运动的PID控制
    // PID pid_d;
    // pid_d.target = 640;
    // pid_d.fdb = x;
    // pid_d.outputMax = 1;
    // pid_d.Kp = 1;
    // pid_d.Ki = 0;
    // pid_d.Kd = 0;    
    // pid_d.err[0] = pid_d.target - pid_d.fdb;
    // pid_d.output = pid_d.Kp*(pid_d.err[0]-pid_d.err[1]) + pid_d.Ki*pid_d.err[0]+pid_d.Kd*(pid_d.err[0]+pid_d.err[2]-2*pid_d.err[1]);
    // pid_d.err[2] = pid_d.err[1];
    // pid_d.err[1] = pid_d.err[0];
    // if(pid_d.output > pid_d.outputMax) pid_d.output = pid_d.outputMax;
    // if(pid_d.output < -pid_d.outputMax) pid_d.output = -pid_d.outputMax;
    // 发布速度信息
    // cmd_red.linear.x = pid_x.output;
    // cmd_red.angular.z = pid_d.output;
    // speed_pub.publish(cmd_red);

    // DEBUG信息显示
    std::cout << "Bouding Boxes (header):" << detect_msg->header << std::endl;
    std::cout << "Bouding Boxes (image_header):" << detect_msg->image_header << std::endl;
    std::cout << "Bouding Boxes (Class):" << "\t";
    for (int i = 0; i < num; i++){
      std::cout << detect_msg->bounding_boxes[i].Class << "\t";
      std::cout << "Center distance :  " << depths[centerIdx] << "  m" << std::endl;
      std::cout << x << std::endl;
    }
    std::cout << "\033[2J\033[1;1H";
  }
};

int main(int argc, char **argv){
  ROS_WARN("*****START*****");
  ros::init(argc, argv, "mechinevision");
  SubscribeAndPublish sub_and_pub;
  return 0;
}
