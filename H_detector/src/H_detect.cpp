#include "H_detector/detector.h"

using namespace cv;
using namespace std;


void DetectH(Mat& frame, int h_min, int h_max, bool _side_flag,
        int& hough_thresh, Rect& rect, vector<double>& LineScope)
{
    bool flag;
    Mat gray;
    Mat cannyEdge;

    ColorSeg(frame, gray, rect, h_min, h_max);

//        //Gaussian Filtering
//        Mat Gaus;
//        Gaus.create(gray.size(), gray.type());
//        GaussianBlur(gray, Gaus, Size(7,7), 4.0);
//
//        //Otsu Thresholding
//        Mat thres_img;
//        thres_img.create(Gaus.size(), CV_8U);
//        threshold(Gaus, thres_img, 0, 255.0, CV_THRESH_BINARY | CV_THRESH_OTSU);
//        imshow("Thresholding", thres_img);

    //Use Canny Operator to extract edge
    cannyEdge.create(gray.size(), CV_8U);
    Canny(gray, cannyEdge, CANNY_EDGE_THRESH, CANNY_EDGE_THRESH*3, 3);

//        //Use thinning to extract skeleton
//        ximgproc::thinning(gray,skeleton);
//        imshow("Skeletons",skeleton);

    //Hough Line Transform P
    vector<Vec4i> lines; // will hold the results of the detection
    HoughLinesP(cannyEdge, lines, 1, CV_PI/180,
                hough_thresh, 20, 3 );

//    //Standard Hough Line Transform
//    vector<Vec2f> lines; // will hold the results of the detection
//    HoughLines(cannyEdge, lines, 1, CV_PI/180, hough_thresh, 0, 0 );

    //cout<<lines.size()<<'\n';
    //Adaptive HoughTransform Threshold
    if(lines.size() > 15)
    {
        hough_thresh++;
        flag = false;
    }
    else if(lines.size() < 8)
    {
        if(hough_thresh > 3)
            hough_thresh--;

        flag = false;
    }
    else
        flag = true;
    //cout<<hough_thresh<<'\n';

    //LineThresh(lines);


    Mat HoughLine;
    cvtColor(cannyEdge, HoughLine, CV_GRAY2BGR);
    DrawHoughLineP(lines, HoughLine);
    //DrawHoughLine(lines, HoughLine);

    if(!lines.empty())
    {
        GetLinesScope(lines, LineScope);
    }

//    //Detect Intersections
//    vector<Point2f> intn;
//    Point2f _get_ints;
//    for(size_t i=0; i<lines.size(); i++){
//        for(size_t j=i; j<lines.size(); j++){
//            if(GetIntersection(frame, lines[i], lines[j], _get_ints))
//            {
//                cout<<_get_ints<<endl;
//                intn.push_back(_get_ints);
//            }
//            else;
//        }
//    }
//
//    if(!intn.empty())
//    {
//        //cout<<intn[0].x<<" "<<intn[0].y<<" "<<intn[1].x<<" "<<intn[1].y<<endl;
//        //cout<<rect.x<<"  "<<rect.y<<endl;
//        //cout<<intn.size()<<endl;
//
//        //cout<<intn[1].x<<" "<<intn[1].y<<" "<<intn[2].x<<" "<<intn[2].y<<endl;
//        for(size_t i=0; i<intn.size(); i++)
//        {
//            putText(frame,"ptr", intn[i],FONT_HERSHEY_SIMPLEX,
//                    1,(255,255,255),2,LINE_AA);
//        }
//    }
//    else;


    if(_side_flag == 0)
    {
        imshow("Left frame", frame);
        imshow("Left thresholding", gray);
        imshow("Left Canny", cannyEdge);
        imshow("Left HoughLine", HoughLine);
        //cvMoveWindow("Hough Line Detected",200,200);
    }
    else if(_side_flag == 1)
    {
        imshow("Right frame", frame);
        imshow("Right thresholding", gray);
        imshow("Right Canny", cannyEdge);
        imshow("Right HoughLine", HoughLine);
        //cvMoveWindow("Hough Line Detected",200,200);
    }
    else;

}

vector<Point2f> bubbleSort(vector<Point2f>& intn, const Rect& rect)
{
    bool swapp = true;
    while(swapp)
    {
        swapp = false;
        for (std::size_t i = 0; i < intn.size() -1; i++)
        {
            if ((abs(intn[i].x - rect.x) + abs(intn[i].y - rect.y)) >
                (abs(intn[i+1].x - rect.x) + abs(intn[i+1].y - rect.y)))
            {
                intn[i] += intn[i+1];
                intn[i+1] = intn[i] - intn[i+1];
                intn[i] -= intn[i+1];
                swapp = true;
            }
        }
    }
}

int Otsu(IplImage* src)
{
    int height = src->height;
    int width = src->width;

    //histogram
    float histogram[256] = { 0 };
    for (int i = 0; i < height; i++)
    {
        unsigned char* p = (unsigned char*)src->imageData + src->widthStep * i;
        for (int j = 0; j < width; j++)
        {
            histogram[*p++]++;
        }
    }
    //normalize histogram
    int size = height * width;
    for (int i = 0; i < 256; i++)
    {
        histogram[i] = histogram[i] / size;
    }

    //average pixel value
    float avgValue = 0;
    for (int i = 0; i < 256; i++)
    {
        avgValue += i * histogram[i];  //整幅图像的平均灰度
    }

    int threshold;
    float maxVariance = 0;
    float w = 0, u = 0;
    for (int i = 0; i < 256; i++)
    {
        w += histogram[i];  //假设当前灰度i为阈值, 0~i 灰度的像素(假设像素值在此范围的像素叫做前景像素) 所占整幅图像的比例
        u += i * histogram[i];  // 灰度i 之前的像素(0~i)的平均灰度值： 前景像素的平均灰度值

        float t = avgValue * w - u;
        float variance = t * t / (w * (1 - w));
        if (variance > maxVariance)
        {
            maxVariance = variance;
            threshold = i;
        }
    }

    return threshold;
}

bool GetIntersection(const Mat& frame, Vec2f LineA, Vec2f LineB, Point2f& intn)
{
    double a1, a2, b1 ,b2;
    a1 = 1/atan(LineA[1]);
    a2 = 1/atan(LineB[1]);

    b1 = LineA[0];
    b2 = LineB[0];

    intn.x = (b2-b1)/(a1-a2);
    intn.y = (a1*b2-a2*b1)/(a1-a2);

    if(!(intn.x>0 && intn.x<(float)frame.cols && intn.y>0 && intn.y<(float)frame.rows))
    {
        intn.x = -1;
        intn.y = -1;
        return false;
    }
    else
    {
        //cout<<intn.x<<"  "<<intn.y<<endl;
        return true;
    }
}

void DrawHoughLineP(vector<Vec4i> lines, Mat& HoughLine)
{
    // Draw the lines
    for( size_t i = 0; i < lines.size(); i++ )
    {
        Vec4i l = lines[i];
        line( HoughLine, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, LINE_AA);
    }
}

void DrawHoughLine(vector<Vec2f> lines, Mat& HoughLine)
{
    // Draw the lines
    for( size_t i = 0; i < lines.size(); i++ )
    {
        float rho = lines[i][0], theta = lines[i][1];
        Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        line(HoughLine, pt1, pt2, Scalar(0,0,255), 3, LINE_AA);
    }

}

void ColorSeg(Mat src, Mat& frame_threshold, Rect& rect, int h_min, int h_max)
{
    Mat frame = src.clone();
    Mat RGB_Gaus = src.clone();
    Mat frame_HSV = src.clone();

    Mat kernelOp, kernelCl;

    kernelOp = getStructuringElement(MORPH_RECT, Size(5, 5));
    kernelCl = getStructuringElement(MORPH_RECT, Size(5, 5));

    std::vector<std::vector<Point> > contours;
    std::vector<Vec4i> hireachy;
    Point2f center;
    float radius=20;

    //Gaussain Filtering
    GaussianBlur(frame, RGB_Gaus, Size(5, 5), 3.0);

    //RGB to HSV
    cvtColor(RGB_Gaus, frame_HSV, CV_RGB2HSV);
    //Color Yellow
    inRange(frame_HSV, Scalar(h_min, 43, 46),
            Scalar(h_max, 255, 255), frame_threshold);
    //imshow("HSVThresh", frame_threshold);

    //开操作
    morphologyEx(frame_threshold,frame_threshold,MORPH_OPEN,kernelOp);
    morphologyEx(frame_threshold,frame_threshold,MORPH_CLOSE,kernelCl);

    //获取边界
    findContours(frame_threshold, contours, hireachy,
            RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0,0));
    //框选面积最大的边界
    if (contours.size() > 0)
    {
        double maxArea=0;
        for (int i = 0; i < contours.size(); i++)
        {
            double area = contourArea(contours[static_cast<int>(i)]);
            if (area > maxArea)
            {
                maxArea = area;
                rect = boundingRect(contours[static_cast<int>(i)]);
                minEnclosingCircle(contours[static_cast<int>(i)], center, radius);
            }
        }
    }

    //矩形框
    rectangle(frame,rect, Scalar(0,255,0),2);
}

void LineThresh(vector<Vec2f>& lines)
{
    int A = 10;
    double B = CV_PI / 10;
    //将多条非常相像的直线剔除
    while(true)
    {
        for (int i = 0; i <lines.size(); i++)
        {
            for (int j = 0; j < lines.size(); j++)
            {
                if (j != i)
                {
                    float rho1 = lines[i][0];
                    float threta1 = lines[i][1];
                    float rho2 = lines[j][0];
                    float threta2 = lines[j][1];
                    if (abs(rho1 - rho2) < A && abs(threta1 - threta2) < B)
                    {
                        lines.erase(lines.begin()+j);
                    }
                }
            }
        }
        if (lines.size() > 8)//剔除一圈后如何直线的数量大于6，则改变A和B，继续删除相似的直线
        {
            A = A + 1;
            B = B + CV_PI / 180;
        }
        else
        {
            //cout<<lines.size()<<'\n';
            break;
        }
    }
}

int CheckLine(vector<Vec4i>& lines)
{
    int _check_line = 0;
    double k[10];
    int _count_left = 0;
    int _count_right = 0;
    int _count_middle = 0;
    for(size_t i = 0; i < lines.size(); i++)
    {
        Vec4i l = lines[i];
        Point pt1(l[0],l[1]);
        Point pt2(l[2],l[3]);
        if((pt1.x-pt2.x) != 0)
            k[i] = (double)(pt1.y-pt2.y)/(double)(pt1.x-pt2.x);
        else
            k[i] = 9999999;  //A large slope
    }
    for(size_t i = 0; i < lines.size(); i++)
    {
        if(k[i]<0)
            _count_left++;
        if(k[i]>0)
            _count_right++;
        if(k[i]>-0.01 && k[i]<0.01)
            _count_middle++;
    }
    cout<<_count_middle<<"  "<<_count_right<<"  "<<_count_left<<'\n'<<endl;
    return _check_line;
}

void GetLinesScope(const vector<Vec4i>& lines, vector<double>& Scope)
{
    for(size_t i=0; i<lines.size(); i++)
    {
        Point2f pt1;
        Point2f pt2;

        pt1.x = lines[i][0];
        pt1.y = lines[i][1];
        pt2.x = lines[i][2];
        pt2.y = lines[i][3];

        if(abs(pt1.x-pt2.x)>10e-5)
        {
            Scope.push_back(atan2((double)(pt1.x-pt2.x),(double)(pt1.y-pt2.y)) * 180 / CV_PI);
        }
        else
            Scope.push_back(90);
    }

    double _scope_cons = 15;
    while(true) {
        for (size_t i=0; i<Scope.size(); i++) {
            for(size_t j=0; j<Scope.size(); j++)
            {
                if(j!=i)
                {
                    if(abs(Scope[j]-Scope[i])<_scope_cons)
                        Scope.erase(Scope.begin() + j);
                }
            }
        }

        if (Scope.size() > 2)//继续删除相似的
        {
            _scope_cons = _scope_cons + 1;

        }
        else{
            break;
        }
    }
}

void SwapVector2(vector<double>& Vector)
{
    if(Vector[0]>Vector[1])
    {
        double swap;
        swap = Vector[1];
        Vector[1] = Vector[0];
        Vector[0] = swap;
    }
    else;

//    cout<<"\n"<<"*********************"<<endl;
//    for(size_t i=0; i<Vector.size(); i++)
//    {
//        cout<<Vector[i]<<endl;
//    }
}