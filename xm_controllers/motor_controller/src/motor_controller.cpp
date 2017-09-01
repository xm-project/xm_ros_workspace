#include <motor_controller/motor_controller.h>
// tan90, all copied
namespace motor_controller
{
    namespace internal
    {    
        std::string getLeafNamespace(const ros::NodeHandle& nh)
        {
            const std::string complete_ns = nh.getNamespace();
            std::size_t id   = complete_ns.find_last_of("/");
            return complete_ns.substr(id + 1);//获取controller name, use for info
        }  
    }//use namespace internal

    void MotorController::preemptActiveGoal()
    {
        RealtimeGoalHandlePtr current_active_goal(rt_active_goal_);
        // cancel the currently active goal
        if (current_active_goal)
        {
            rt_active_goal_.reset();
            if (current_active_goal->gh_.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
                current_active_goal->gh_.setCanceled();
        }
    }
    // fun init
    bool MotorController::init(hardware_interface::PositionJointInterface * gh,
                                    ros::NodeHandle& root_nh,
                                    ros::NodeHandle& controller_nh)
    {
        using namespace internal;
        //cache(缓存) controller node handle
        controller_nh_ = controller_nh;
        // get controller name
        name_ = getLeafNamespace(controller_nh_);
        // action status checking update rate
        double action_monitor_rate = 20.0;
        controller_nh_.getParam("action_monitor_rate", action_monitor_rate);
        action_monitor_period_ =  ros::Duration(1.0/ action_monitor_rate);
        ROS_DEBUG_STREAM_NAMED(name_, "MotorControllerAction status changes will be monitored at " << action_monitor_rate << "Hz.");

        // controller joint 
        // controller_nh_.getParam("joint", joint_name_);
        // here we just want to control the Joint_lifting 
        joint_name_.push_back("Joint_lifting");

        if (joint_name_.empty())
        {
            ROS_ERROR_STREAM_NAMED(name_, "could not find joint name on param server!!!");
            return false;
        }

        // init members of class
        // joint handle
        try
        {
            for (int i=0;i<joint_name_.size();i++)
            {
                joint_handle_.push_back(gh->getHandle(joint_name_[i]));
            }
           
        }
        catch(...)
        {
            ROS_ERROR_STREAM_NAMED(name_, "could not find joint .");
            return false;
        }
        ROS_DEBUG_STREAM_NAMED(name_, "Initialized controller '" << name_ << "' with:" <<
			 "\n- Hardware interface type: '" <<this->getHardwareInterfaceType() << "'" <<
			 "\n");

        //result api
        // pre_alloc_result_.reset(new xm_msgs::xm_GripperBoolResult());
        pre_alloc_result_.reset(new control_msgs::FollowJointTrajectoryResult());
        // data init?? tan90
       
        success_value = false;
        // action interface
        action_server_.reset(new ActionServer(controller_nh_, "lifting_cmd",
                            boost::bind(&MotorController::goalCB, this, _1),
                            boost::bind(&MotorController::cancelCB, this , _1),
                            false));
        action_server_->start();
     
        return true;
    }
    // rt read non-rt write
    void MotorController::update(const ros::Time& time, const ros::Duration& period)
    {
            command_struct_rt_ = *(command_.readFromRT());
            if (!success_value) return;
            joint_handle_[0].setCommand(command_struct_rt_);
                
            // pub result
            rt_active_goal_->setSucceeded(pre_alloc_result_);  
            success_value =false;  
            // smooth ?? tan90
    }

     void MotorController::goalCB(GoalHandle gh)
    {
        ROS_DEBUG_STREAM_NAMED(name_, "receive new action goal");
        // controller is not running
        if (!isRunning())
        {
            ROS_ERROR_NAMED(name_, "cannot accept new action goals, controller is stop!!");
            // xm_msgs::xm_GripperBoolResult result;
            control_msgs::FollowJointTrajectoryResult result;
            gh.setRejected(result);
            return ;
        }
        // try to update goal
        RealtimeGoalHandlePtr rt_goal(new RealtimeGoalHandle(gh));
        // accept new goal
        preemptActiveGoal();
        gh.setAccepted();
        // 1 for open pose = 0
        // 0 for close pose =-0.03
        success_value =true;
        //goal_bool =true is open
        double position_lifting;
        // point ->the joint-trajectory number
        // position ->the joint number
        position_lifting = gh.getGoal()->trajectory.points.back().positions[0];
        

        command_struct_ = position_lifting;
        //NONRT write data to real_timebuffer 
        command_.writeFromNonRT(command_struct_);
       
        // Setup goal status checking timer
        // 创建一个定时器，每隔action_monitor_period_时间检查状态？
        goal_handle_timer_ = controller_nh_.createTimer(action_monitor_period_,
                            &RealtimeGoalHandle::runNonRealtime,
                            rt_goal);
        goal_handle_timer_.start();
        rt_active_goal_ = rt_goal;

    }

    void MotorController::cancelCB(GoalHandle gh)
    {
        RealtimeGoalHandlePtr current_active_goal(rt_active_goal_);
        
        // Check that cancel request refers to currently active goal (if any)
        if (current_active_goal && current_active_goal->gh_ == gh)
        {
            // Reset current goal
            rt_active_goal_.reset();
            
            // Enter hold current position mode
           
            ROS_DEBUG_NAMED(name_, "Canceling active action goal because cancel callback recieved from actionlib.");
            
            // Mark the current goal as canceled
            current_active_goal->gh_.setCanceled();
        }
    }

    void MotorController::starting(const ros::Time& time)
    {
       
        joint_handle_[0].setCommand(0.0);
        command_struct_rt_ = 0.0;
        command_.initRT(command_struct_rt_);
        state_ = RUNNING;
        ROS_ERROR("Started xm_robot lifting_controller");

    }
    void MotorController::stopping(const ros::Time& time)
    {
        preemptActiveGoal();
        ROS_ERROR("byebye");
    }
}
