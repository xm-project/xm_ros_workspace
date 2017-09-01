# xm_arm_packages

Create Date: 2017.8.30

Function: 
晓萌第二代机械臂的代码

Package:1.libfreenect2_ros_grabber: 从Kinect2获取点云数据并以topic形式发布，用来生成Octomap，实现机械臂避障
        2.xm_arm_nav: 所有使用的机械臂代码，通过在代码里设计小型状态机实现接口简单化
        3.xm_moveit_config: moveit配置文件

Summary:
   在这个版本的机械臂代码里，基本把moveit关于motion-plan以及障碍物检测的特性用起来了，但是还存在以下问题：
    1.所有代码是用python实现，只能调用相当少的moveit特性（Python的moveit-API特别少）
    2.motion-plan成功率不够，目前实现的避障只是简单的避免了机械臂和障碍物的碰撞，无法实现任意（可行）位置的可行规划
    3.motion-plan规划路径比较诡异，应该需要进一步调整参数
    4.motion-plan和ik解算都只是考虑了机械臂本体，没有发挥移动机械臂的优势，即把底盘也加入规划或者ik中（但是这样可能需要调用一些导航的接口）
    5.moveit基本特性用起来了，但是能用的还有很多。。。

Use:
运行xm_arm_nav包中的节点， 首先需要
    (1) roslaunch handfree_hw xm_bot.launch, 启动handsfree_node 和 move_group节点
    (2) roslaunch xm_arm_nav xm_arm.launch 启动 obstacles_gen 和 xm_ik_server节点
各个节点的使用：
    见脚本文件注释