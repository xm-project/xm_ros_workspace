#ifndef MOTOR_CONTROLLER_H_
#define MOTOR_CONTROLLER_H_

//c++
#include <cassert>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>
#include <math.h>
// Boost
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>

// ROS
#include <ros/node_handle.h>
#include <ros/ros.h>

// ROS services
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <control_msgs/QueryTrajectoryState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryResult.h>
// actionlib
#include <actionlib/server/action_server.h>

// ros_controls
#include <realtime_tools/realtime_server_goal_handle.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/internal/demangle_symbol.h>
#include <realtime_tools/realtime_buffer.h>
#include <pluginlib/class_list_macros.h>
#include <tf/tfMessage.h>

// 这个控制器是根据ros-controllers提供的官方控制器joint_trajectory_controller修改来的
// 众所周知，joint_trajectory_controller实现了对多个关节结构（机械臂）控制时，使用样条函数来生成轨迹
// 但是底层对升降的控制策略似乎有问题，升降电机的启动和停止并没有遵循机械臂关节应有的动力学要求（仔细观察一下机械臂电机和升降电机工作行为即可发现）
// 所以把升降单独使用了一个控制器，解决了升降电机抖动严重的问题，但是实际上是否已经造成了升降和机械臂其他关节的不协调？

namespace motor_controller
{
    class MotorController : public controller_interface::Controller<hardware_interface::PositionJointInterface>
    {
        public:
            MotorController()
            {
            }
            virtual ~MotorController()
            {
            }

            bool init(hardware_interface::PositionJointInterface *jh,
                        ros::NodeHandle& root_nh,
                        ros::NodeHandle& controller_nh);
            void update(const ros::Time &time, const ros::Duration& period);

            void starting(const ros::Time& time);

            void stopping(const ros::Time& time);
        private:
            typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>                  ActionServer;
            typedef boost::shared_ptr<ActionServer>                                                     ActionServerPtr;
            typedef ActionServer::GoalHandle                                                            GoalHandle;
            typedef realtime_tools::RealtimeServerGoalHandle<control_msgs::FollowJointTrajectoryAction> RealtimeGoalHandle;
            typedef boost::shared_ptr<RealtimeGoalHandle>                                               RealtimeGoalHandlePtr;
            //joint
            std::vector<hardware_interface::JointHandle> joint_handle_;                          ///< Handles to controlled joints.
            std::string                     name_;                      ///< Controller names.
            std::vector<std::string>        joint_name_;                   //joints name
            RealtimeGoalHandlePtr                        rt_active_goal_;     ///< Currently active action goal, if any.
           
            // xm_msgs::xm_GripperBoolResultPtr pre_alloc_result_;
            control_msgs::FollowJointTrajectoryResultPtr pre_alloc_result_;
            ros::Duration action_monitor_period_;
            
            ros::NodeHandle    controller_nh_;
            ActionServerPtr    action_server_;
            ros::Timer         goal_handle_timer_;
            bool success_value;
            std::vector<std::string> base_name_;
          
            realtime_tools::RealtimeBuffer<double> command_;
            double command_struct_, command_struct_rt_;
            
            void goalCB(GoalHandle gh);
            void cancelCB(GoalHandle gh);
            void preemptActiveGoal();
            
    };
PLUGINLIB_EXPORT_CLASS(motor_controller::MotorController, controller_interface::ControllerBase)

}

#endif