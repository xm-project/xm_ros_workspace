#include <gripper_controller/gripper_controller.h> 
namespace gripper_controller
{
    namespace internal
    {
            
        std::string getLeafNamespace(const ros::NodeHandle& nh)
        {
            const std::string complete_ns = nh.getNamespace();
            std::size_t id   = complete_ns.find_last_of("/");
            return complete_ns.substr(id + 1);//获取controller name
        }  
        
    }// use namespace internal

    void GripperController::preemptActiveGoal()
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

    //fun init
    bool GripperController::init(hardware_interface::GripperCmdInterface * gh,
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
        ROS_DEBUG_STREAM_NAMED(name_, "GripperControllerAction status changes will be monitored at " << action_monitor_rate << "Hz.");

        // controller joint 
        // controller_nh_.getParam("joint", joint_name_);
        joint_name_.push_back("Joint_left_finger");
        joint_name_.push_back("Joint_right_finger");

        if (joint_name_.empty())
        {
            ROS_ERROR_STREAM_NAMED(name_, "could not find joint name on param server!!!");
            return false;
        }

        // init members of class
        // joint handle
        try
        {
            for (int i=0;i<2;i++)
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
        pre_alloc_result_.reset(new control_msgs::GripperCommandResult());
       
        pre_alloc_result_->position =0.0;
        pre_alloc_result_->effort = 0.0;
       
        success_value = false;
        // action interface
        action_server_.reset(new ActionServer(controller_nh_, "gripper_cmd",
                            boost::bind(&GripperController::goalCB, this, _1),
                            boost::bind(&GripperController::cancelCB, this , _1),
                            false));
        action_server_->start();
     
        return true;
    }

    void GripperController::update(const ros::Time& time, const ros::Duration& period)
    {
            command_struct_rt_ = *(command_.readFromRT());
            bool temp_cmd = command_struct_rt_;
            // 1 for open 
            // 0 for close
            if (!success_value) return;
            if(!temp_cmd)
            {
                joint_handle_[0].setGripperCmd(false);
                joint_handle_[1].setGripperCmd(false);
                
                joint_handle_[0].setCmd(-0.01);   
                joint_handle_[1].setCmd(-0.01);   
                
            }
            
            else 
            {
                joint_handle_[0].setGripperCmd(true);
                joint_handle_[1].setGripperCmd(true);

                joint_handle_[0].setCmd(0.0);
                joint_handle_[1].setCmd(0.0);
                
            }
            // pub result
            pre_alloc_result_->position =0.0;
            pre_alloc_result_->effort = 0.0;
            rt_active_goal_->setSucceeded(pre_alloc_result_);  
            success_value =false;  
       
    }
    
    void GripperController::goalCB(GoalHandle gh)
    {
        ROS_DEBUG_STREAM_NAMED(name_, "receive new action goal");
        // controller is not running
        if (!isRunning())
        {
            ROS_ERROR_NAMED(name_, "cannot accept new action goals, controller is stop!!");
            // xm_msgs::xm_GripperBoolResult result;
            control_msgs::GripperCommandResult result;
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
        float position_gripper = gh.getGoal()->command.position;
        float max_effort_gripper = gh.getGoal()->command.max_effort;
        // due to the use of grippercommandaction interface
        if (position_gripper>-0.01&&position_gripper<0.01)
        {
            command_struct_ = true;
        }
        else
        { 
            command_struct_  =false;
        }
       
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

    void GripperController::cancelCB(GoalHandle gh)
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

    void GripperController::starting(const ros::Time& time)
    {
        joint_handle_[0].setGripperCmd(true);//1kai 0he
        joint_handle_[1].setGripperCmd(true);//1kai 0he
        
        joint_handle_[0].setCmd(0.0);
        joint_handle_[1].setCmd(0.0);
        
        command_struct_rt_ = true;
        command_.initRT(command_struct_rt_);
        state_ = RUNNING;
        ROS_ERROR("Started xm_robot gripper_controller");

    }
    void GripperController::stopping(const ros::Time& time)
    {
        preemptActiveGoal();
        ROS_ERROR("OH! FUCK!!HEHEHE");
    }
}
