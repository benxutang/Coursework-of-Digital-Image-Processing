#include "detector/detector.h"

using namespace cv;
using namespace std;
using namespace message_filters;

class SubscribeAndPublish{
public:
    ros::NodeHandle n;
    ros::Publisher pub;
    SubscribeAndPublish(){
        ros::Rate loop_rate(100);
        //pub = n.advertise<geometry_msgs::Twist>("/controler", 5);
        pub = n.advertise<geometry_msgs::Twist>("/smoother_cmd_vel", 2);
        message_filters::Subscriber<sensor_msgs::Image> sub1(n, "/zed/zed_node/rgb_raw/image_raw_color", 1);
        message_filters::Subscriber<sensor_msgs::Image> sub2(n, "/zed/zed_node/depth/depth_registered", 1);
        
        typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> syncPolicy;
        Synchronizer<syncPolicy> sync(syncPolicy(10), sub1, sub2);
        sync.registerCallback(boost::bind(&SubscribeAndPublish::messagecallback, this, _1, _2));
        ros::spin();
    }

    void messagecallback(const sensor_msgs::Image::ConstPtr &msg1, const sensor_msgs::Image::ConstPtr &msg2)    
    {
            static int count = 0;
            static vector<Point2f> Angle;
            static vector<double> Depth;
            static clock_t begin;
            static bool checkonce = true;
            static double angle1 = 0, angle2 = 0;
            static clock_t runtime;
            static double duration;
            static int centerIdx;
            static double depth_data;
            static double depth_data_avg;
            static int hough_thresh = HOUGH_THRESH_INIT;

            static bool IsDrivenNear = false;
            static int countStop = 0;

            static geometry_msgs::Twist cmd_red;

            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg1, sensor_msgs::image_encodings::BGR8);
            Mat frame_left = cv_ptr->image;
            Rect rect;

            float *depths = (float *)(&msg2->data[0]); // 定义照片深度信息


            if(IsDrivenNear != true)
            {
                vector<double> LineScope;
                DetectH(frame_left, H_MIN_LEFT, H_MAX_LEFT, 0,
                        hough_thresh, rect, LineScope);
                
                if(rect.x !=0 && rect.y != 0)
                {
                    ROS_INFO("Moving Close to H ...");
                    
                    if(rect.x<= frame_left.cols/2)
                    {
                        //cmd_red.linear.x = 0;
                        cmd_red.angular.z = abs(rect.x - frame_left.cols/2)*0.003;
                        cout<<abs(rect.x - frame_left.cols/2)*0.001<<endl;
                    }
                    else if(rect.x>= frame_left.cols/2)
                    {
                        //cmd_red.linear.x = 0;
                        cmd_red.angular.z = -abs(rect.x - frame_left.cols/2)*0.003;
                        cout<<abs(rect.x - frame_left.cols/2)*0.001<<endl;
                    }


                    if(rect.x<= frame_left.cols/2 + 20 && rect.x>= frame_left.cols/2 - 20)
                    {
                        if(rect.y <= frame_left.rows*0.6)
                        {
                            cmd_red.linear.x = abs(rect.y - frame_left.rows*0.75)*0.001;
                            cmd_red.angular.z = 0;
                            //cout<<abs(rect.y - frame_left.rows*0.6)<<endl;
                        }
                        // else if(rect.y <= 150 && rect.y > 250)
                        // {
                        //     cmd_red.linear.x = 0.1;
                        //     cmd_red.angular.z = 0;
                        // }
                        else if(rect.y > frame_left.rows*0.6-20)
                        {
                            countStop++;
                            if(countStop > 5)
                            {
                                ROS_WARN("*****Stop*****");
                                IsDrivenNear = true;
                                cmd_red.linear.x = 0;
                                cmd_red.angular.z = 0;                            
                            }
                        }
                        
                    }
                    pub.publish(cmd_red);
                }

                waitKey(2);
            }
            
            else
            {
                //ROS_INFO("Detecting H");
                if (count < 25)
                {
                    vector<double> LineScope;
                    DetectH(frame_left, H_MIN_LEFT, H_MAX_LEFT, 0,
                            hough_thresh, rect, LineScope);
                    //DetectH(frame_right, H_MIN_RIGHT, H_MAX_RIGHT, 1);
                    if (LineScope.size() > 1){
                        SwapVector2(LineScope);
                        Point2f angle;
                        angle.x = LineScope[0];
                        angle.y = LineScope[1];
                        if (abs(angle.x - angle.y) > 20){
                            Angle.push_back(angle);
                        }
                    }

                    centerIdx = rect.x + msg2->width * rect.y;
                    depth_data = depths[centerIdx];

                    if(0<depth_data && depth_data< _MAX_DEPTH_DATA)
                    {
                        //cout << rect.x << "  " << rect.y << "  " << depth_data << endl;
                        depth_data_avg = sqrt(depth_data*depth_data - _CAM_HIGHT*_CAM_HIGHT)+0.2;
                        //Depth.push_back(sqrt(depth_data*depth_data - _CAM_HIGHT*_CAM_HIGHT));
                    }
                    else;
                    
                    waitKey(2);
                    count++;
                }

                else{
                    if (checkonce == true){
                        //Averaging
                        for (size_t i = 0; i < Angle.size(); i++){
                            angle1 += Angle[i].x / Angle.size();
                            angle2 += Angle[i].y / Angle.size();
                        }

                        // for (size_t i = 0; i < Depth.size(); i++)
                        // {
                        //     depth_data_avg = Depth[i]/Depth.size();
                        // }

                        begin = clock();
                        cout << angle1 << "  " << angle2 << "  " << depth_data_avg <<endl;
                        checkonce = false;
                    }

                    runtime = clock();
                    duration = (double)(runtime - begin) / CLOCKS_PER_SEC;
                    //geometry_msgs::Twist cmd_red;

                    if (duration <= (depth_data_avg / LINEAR_X / 10)){
                        cmd_red.linear.x = LINEAR_X;
                        cmd_red.angular.z = 0;
                        pub.publish(cmd_red);
                    }

                    else if ((depth_data_avg / LINEAR_X / 10) < duration && duration <= ((depth_data_avg / LINEAR_X / 10) + (180 + angle2) * 25 / 3600)){
                        //cout<<angle1<<endl;
                        cmd_red.linear.x = 0;
                        cmd_red.angular.z = -ANGULAR;
                        pub.publish(cmd_red);
                    }

                    else
                    {
                        //cout << duration << endl;
                        cmd_red.linear.x = 0;
                        cmd_red.angular.z = 0;
                        pub.publish(cmd_red);
                    }
                }/* code */
            }
    }
};

int main(int argc, char **argv)
{
    ROS_WARN("*****START*****");
    ros::init(argc, argv, "Detector");
    SubscribeAndPublish sub_and_pub;
    return 0;
}
