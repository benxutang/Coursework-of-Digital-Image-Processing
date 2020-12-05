#include "H_detector/detector.h"

using namespace cv;
using namespace std;

int main(int argc, char*argv[])
{
    VideoCapture capture;
    capture.open(0); // 1 为打开 zed 相机, 0 为打开笔记本摄像头
    ROS_WARN("*****START");
    ros::init(argc,argv,"trafficLaneTrack");//初始化 ROS 节点
    ros::NodeHandle n;
    ros::Rate loop_rate(10);//定义速度发布频率
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/smoother_cmd_vel", 5);

    if (!capture.isOpened())
    {
        printf("摄像头没有正常打开,重新插拔工控机上当摄像头\n");
        return 0;
    }
    waitKey(1000);

    Mat frame_zed, frame_left, frame_right;//当前帧图片
//    Mat skeleton;

    int hough_thresh = HOUGH_THRESH_INIT;

    while (ros::ok())
    {
        capture.read(frame_zed);
        if(frame_zed.empty())
        {
            break;
        }

        //截取zed的左目图片
        frame_left = frame_zed(cv::Rect(0, 0, frame_zed.cols / 2, frame_zed.rows));
        frame_right = frame_zed(cv::Rect(frame_zed.cols/2, 0, frame_zed.cols / 2, frame_zed.rows));

        DetectH(frame_left, H_MIN_LEFT, H_MAX_LEFT, 0, hough_thresh);
        //DetectH(frame_right, H_MIN_RIGHT, H_MAX_RIGHT, 1);

        geometry_msgs::Twist cmd_red;
        // 车的速度值设置
        cmd_red.linear.x = LINEAR_X;
        cmd_red.linear.y = 0;
        cmd_red.linear.z = 0;
        cmd_red.angular.x = 0;
        cmd_red.angular.y = 0;
        cmd_red.angular.z = ANGULAR;
        pub.publish(cmd_red);

        ros::spinOnce();
        waitKey(5);
    }
    return 0;
}