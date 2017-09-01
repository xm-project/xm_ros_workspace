#ifndef GRIPPER_CONTROLLER_H_
#define GRIPPER_CONTROLLER_H_
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
// #include <xm_msgs/xm_GripperBoolAction.h>
#include <control_msgs/GripperCommandAction.h>
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
#include <handsfree_hw/gripper_cmd_interface.h>
// 这个控制器是我仿照ros官方提供的ros-controllers中的gripper_action_controller修改的简易版本
// TODO:如果以后需要添加力矩控制，建议直接使用gripper_action_controller即可
// TODO:其实如果爪子还是简单的交给底层去控制的话，建议可以把action接口改为xm_GripperCommand，可以使代码更简单。。。

namespace gripper_controller{

    class  GripperController : public controller_interface::Controller<hardware_interface::GripperCmdInterface>
    {
        public:
            GripperController()
            {
            }
            virtual ~GripperController()
            {
            }

            bool init(hardware_interface::GripperCmdInterface * gh,
                        ros::NodeHandle& root_nh,
                        ros::NodeHandle& controller_nh);
            void update(const ros::Time &time, const ros::Duration& period);

            void starting(const ros::Time& time);

            void stopping(const ros::Time &time);
        private:
           
            typedef actionlib::ActionServer<control_msgs::GripperCommandAction>                         ActionServer;
            typedef boost::shared_ptr<ActionServer>                                                     ActionServerPtr;
            typedef ActionServer::GoalHandle                                                            GoalHandle;
            typedef realtime_tools::RealtimeServerGoalHandle<control_msgs::GripperCommandAction>        RealtimeGoalHandle;
            typedef boost::shared_ptr<RealtimeGoalHandle>                                               RealtimeGoalHandlePtr;
            //joint
            std::vector<hardware_interface::GripperCmdHandle> joint_handle_;                          ///< Handles to controlled joints.
            std::string                     name_;                      ///< Controlled joint names.
            std::vector<std::string>        joint_name_;
            RealtimeGoalHandlePtr                        rt_active_goal_;     ///< Currently active action goal, if any.
           
            // xm_msgs::xm_GripperBoolResultPtr pre_alloc_result_;
            control_msgs::GripperCommandResultPtr pre_alloc_result_;
            ros::Duration action_monitor_period_;
            
            ros::NodeHandle    controller_nh_;
            ActionServerPtr    action_server_;
            ros::Timer         goal_handle_timer_;
            bool success_value;
            std::vector<std::string> base_name_;
          
            realtime_tools::RealtimeBuffer<bool> command_;
            bool command_struct_, command_struct_rt_;
            
            void goalCB(GoalHandle gh);
            void cancelCB(GoalHandle gh);
            void preemptActiveGoal();
           

         

            double stall_timeout_, stall_velocity_threshold_;                 ///< Stall related parameters
          
            

    };
PLUGINLIB_EXPORT_CLASS(gripper_controller::GripperController, controller_interface::ControllerBase)

}

#endif