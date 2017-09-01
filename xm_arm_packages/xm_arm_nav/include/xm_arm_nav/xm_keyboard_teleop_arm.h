#ifndef XM_KEYBOARD_TELEOP_ARM_H
#define XM_KEYBOARD_TELEOP_ARM_H

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>

#include <ros/ros.h>
#include <xm_msgs/xm_PickOrPlace.h>
#include <boost/thread/thread.hpp>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#define KEYCODE_A 0X61
#define KEYCODE_D 0X64
#define KEYCODE_S 0X73
#define KEYCODE_W 0X77
#define KEYCODE_Q 0X71
#define KEYCODE_E 0X65

// 键盘控制程序
// Q:UP
// E:DOWN
// W:FRONT
// S:BACK
// A:RIGHT
// D:LEFT

class XMKeyboardTeleopNode
{
    private:
        ros::NodeHandle xm_nh_;
        ros::ServiceClient arm_pos_client_;

        xm_msgs::xm_PickOrPlace arm_pos_;
        tf::TransformListener listener;

        


    public:
        XMKeyboardTeleopNode();
        ~XMKeyboardTeleopNode();
        void KeyboardLoop();
        void StopArm();
};
#endif
