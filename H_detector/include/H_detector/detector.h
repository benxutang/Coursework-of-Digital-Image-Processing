#include <cstdio>
#include <cmath>
#include "opencv2/imgproc.hpp"
#include <opencv2/ximgproc.hpp>
#include "opencv2/imgcodecs.hpp"
#include <cv.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include<geometry_msgs/Twist.h>
#include "sensor_msgs/Image.h"

#define max_corners 4
#define LINEAR_X 0
#define ANGULAR 0

#define C CV_PI /3
#define CANNY_EDGE_THRESH 1
#define _MAX_INT_PT 10   //Max intersection point
#define H_MIN_LEFT 75
#define H_MAX_LEFT 90
#define H_MIN_RIGHT 75
#define H_MAX_RIGHT 90
#define HOUGH_THRESH_INIT 8

using namespace cv;
using namespace std;

void DetectH(Mat& frame, int h_min, int h_max, bool _side_flag, int& hough_thresh);

vector<Point2f> bubbleSort(vector<Point2f>& intn, const Rect& rect);

int Otsu(IplImage* src);

bool GetIntersection(const Mat& frame, Vec2f LineA, Vec2f LineB, Point2f& intn);

void DrawHoughLineP(vector<Vec4i> lines, Mat& HoughLine);

void DrawHoughLine(vector<Vec2f> lines, Mat& HoughLine);

void ColorSeg(Mat src, Mat& frame_threshold, Rect& rect, int h_min, int h_max);

void LineThresh(vector<Vec2f>& lines);

int CheckLine(vector<Vec4i>& lines);

void GetLinesScope(const vector<Vec4i>& lines, vector<double>& Scope);