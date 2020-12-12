# Coursework of Digital Image Processing
[![Generic badge](https://img.shields.io/badge/version-v4.1.2-blue.svg)](https://shields.io/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT) 

Coursework of Digital Image Processing

# Pre-required package
- [darknet](https://github.com/pjreddie/darknet)

# Installation
## Install darknet
```
git clone https://github.com/pjreddie/darknet.git
cd darknet
make
```
- Darknet installation: https://pjreddie.com/darknet/install/
- Darknet introduction: https://pjreddie.com/darknet/yolo/

## Install "H" detecting package
- Create catkin workspace
```
mkdir -p ./ImageProcessing_ws/src
cd ./ImageProcessing_ws/src
```
```
git clone https://github.com/TANGBEN7/Coursework-of-Digital-Image-Processing
```
- Make catkin package
```
catkin_make
source ./devel/setup.sh
```
Basic ROS package usage please refer to [ROS Wiki](http://wiki.ros.org/ROS/Tutorials)

# Run ROS node
Starting ROS
```
roscore
```
Run the following ROS nodes in new terminals
```
rosrun 
```
Or use ROS launch

# Reference
## YOLO V3
YOLO is a real-time object detection system. Please refer to YOLO Website: https://pjreddie.com/darknet/yolo/
- How to train your own classifier: https://pjreddie.com/darknet/train-cifar/
> `image_test` is a my package to train my classfier to detect "H"
## YOLO for ROS
`darknet_ros`
Please refer to the README.md on [GitHub](https://github.com/leggedrobotics/darknet_ros)
## Others
Basic Image Processing algorithm. 
- Basic spacial and point processing: Threshloding, Guassian Filtering
- Image Segmentation: Hough Transform, Canny Operator
- Color Image Processing
- K-means
