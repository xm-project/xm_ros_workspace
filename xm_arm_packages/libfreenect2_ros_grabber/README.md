# libfreenect2_ros_grabber package
Simple Kinect 2 / xbox One kinect ros point cloud2 publisher based on libfreenect2pclgrabber:
https://github.com/cpaxton/libfreenect2pclgrabber/blob/master/test.cpp

Please install libfreenect2 before using this: 
https://github.com/OpenKinect/libfreenect2

To build:
1. Place the source folder to your workspace folder.
2. Run catkin build libfreenect2_ros_grabber

To run:
Use roslaunch libfreenect2_ros_grabber libfreenect2.launch

#we use this package to generate the octomap used for the moveit to motion-plan for the 4-dof arm
