# Coursework of Digital Image Processing
[![Generic badge](https://img.shields.io/badge/version-v5.1.1-blue.svg)](https://shields.io/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT) 

Coursework of Digital Image Processing

# Pre-required package
- [darknet](https://github.com/pjreddie/darknet)

# Installation
## Install darknet
Darknet is required for training
```
git clone https://github.com/pjreddie/darknet.git
cd darknet
make
```
- Darknet installation: https://pjreddie.com/darknet/install/
- Darknet introduction: https://pjreddie.com/darknet/yolo/

## Install my marker "H" detection package
- Create catkin workspace
```
mkdir -p ./ImageProcessing_ws/src
cd ./ImageProcessing_ws/src
```
```
git clone https://github.com/TANGBEN7/Coursework-of-Digital-Image-Processing
```
- Build catkin package
```
cd ..
catkin_make
source ./devel/setup.sh
```

# Run ROS node
Start ROS core
```
roscore
```
## Detection based on basic image processing algorithms
```
roslaunch zed_wrapper zed.launch
roslaunch dashgo_driver driver.launch
rosrun test detector
```
![](https://github.com/TANGBEN7/Coursework-of-Digital-Image-Processing/blob/main/assets/img/Frame_1.png)
![](https://github.com/TANGBEN7/Coursework-of-Digital-Image-Processing/blob/main/assets/img/Left%20HoughLine_1.png)

## Detection based on YOLO V3
```
roslaunch zed_wrapper zed.launch
roslaunch dashgo_driver driver.launch
roslaunch darknet_ros darknet_ros.launch
rosrun machinevision subscriber
```

使用Yolo V3，能够识别不完整的“H”标志，这也是我们在用基本的图像处理算法成功提取“H”的位姿信息后，进一步使用Yolo V3的原因，即提升检测的鲁棒性。

- 被雪糕筒挡住一部分的“H”标志

![](https://github.com/TANGBEN7/Coursework-of-Digital-Image-Processing/blob/main/assets/img/YOLO%20V3_6.png)

- 被折起一角的“H”标志

![](https://github.com/TANGBEN7/Coursework-of-Digital-Image-Processing/blob/main/assets/img/YOLO%20V3_3.png)

> 由于当时做课设的时间紧迫，我们就直接用了"person"这个label去代表检测到的"H"标志，所以上面检测出来的“H”都是“person”的标签

# Reference
## YOLO V3
YOLO is a real-time object detection system. Please refer to YOLO Website: https://pjreddie.com/darknet/yolo/
- How to train your own classifier: https://pjreddie.com/darknet/train-cifar/
> `image_test` is a package to train our classfier to detect "H"
## YOLO for ROS
`darknet_ros`
Please refer to the README.md on [GitHub](https://github.com/leggedrobotics/darknet_ros)
## ZED Stereo Camera
- Documentation: https://www.stereolabs.com/docs/
- ZED Node and published topic: https://www.stereolabs.com/docs/ros/zed-node/

## Others
Algorithms below are applied. 
- Basic spacial and point processing: Threshloding, Guassian Filtering
- Image Segmentation: Hough Transform, Canny Operator
- Color Image Processing
- K-means
