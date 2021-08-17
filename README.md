# PLI-SLAM: A Stereo Visual-Inertial SLAM based on Point-Line Features
PLI-SLAM is developed on the foundation of [PL-SLAM](http://github.com/rubengooj/pl-slam) and [ORB_SLAM3](http://github.com/UZ-SLAMLab/ORB_SLAM3), line features are fully engaged in the whole process of the system including tracking, map building and loop detection.
Higher accuracy has been shown in PLI-SLAM on EuRoC Dataset compared to other VIO or point-line based SLAM systems. PLI-SLAM also performed stronger tracking robustness in various real scenario experiments conduted with a self-developed robot. 

**Video demo**: [youtube](https://youtu.be/UuJuEZoYsso)

## 1. License:
The provided code is published under the General Public License Version 3 (GPL v3).
## 2. Prerequisites:
### Ubuntu
Ubuntu 16.04 or higher version is required.
### ROS
We use ROS Melodic for real scenario experiments.
### C++11 or C++0x Compiler
We use the new thread and chrono functionalities of C++11.
### Pangolin
We use Pangolin for visualization and user interface. It can be found at: https://github.com/stevenlovegrove/Pangolin.
### OpenCV
We use OpenCV 3.3.1 to manipulate images and features. 
Noted that [line_descripter](https://github.com/opencv/opencv_contrib/tree/master/modules/line_descriptor) should be impoerted from the [OpenCV/contrib](https://github.com/opencv/opencv_contrib) library independently.
### Eigen3
We use Eigen 3.3.4. It can be found at: http://eigen.tuxfamily.org.
### DBoW2 and g2o
We use modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition and [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations.
## 3. Building PLI-SLAM library and examples on ROS
1. Clone the repository and add the path to the ROS_PACKAGE_PATH environment variable:
```Bash
git clone https://github.com/VealFang/PLI-SLAM.git
gedit ~/.bashrc #Open .bashrc file
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/ORB_SLAM3/Examples/ROS #Add at the end of the file
source ~/.bashrc
```
2. Execute ``build_ros.sh`` script:
```Bash
cd PLI_SLAM
chmod +x build_ros.sh
./build_ros.sh
```
## 4. Run on EuRoC dataset
Download a [rosbag](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) from the EuRoC dataset.
Open 3 tabs on the terminal and run the following command at each tab:  
```Bash
roscore
``` 
```Bash
roslaunch PLI_SLAM Stereo_Inertial Vocabulary Examples/Stereo-Inertial/EuRoC.yaml true
```  
```Bash
rosbag play ROSBAG_PATH 
```
