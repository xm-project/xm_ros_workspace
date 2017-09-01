
// c++
#include <iostream>
#include <cmath>
// ros
#include <ros/ros.h>
// xm_msgs
#include <xm_msgs/xm_SolveIK.h>
#include <xm_msgs/xm_PickOrPlace.h>
// moveit_demo use c++
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit/kinematic_constraints/utils.h>
#include <eigen_conversions/eigen_msg.h>

// 基于moveit-tutorial 修改的抓取demo
class xm_pick_or_place
{
public:
	xm_pick_or_place(ros::NodeHandle &n )
		:nh(n)
	{
		xm_ik_client = nh.serviceClient<xm_msgs::xm_SolveIK>("xm_ik_solver");
		arm_server = nh.advertiseService("xm_pick_or_place", &xm_pick_or_place::service_callback,this);
		display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

	}
	~xm_pick_or_place()
    {
    }
private:

	bool service_callback(xm_msgs::xm_PickOrPlaceRequest &req,xm_msgs::xm_PickOrPlaceResponse &res)//"xm_pick_or_place"
	{
		xm_msgs::xm_SolveIK srv;
		if(req.action ==0)//nav_pose with gripper open
		{
				res.result=prepare_nav();
				gripper_exe(true);
		}
		if(req.action==1)//pick
		{
			srv.request.goal = req.goal_position;
			
			// srv.request.goal.header.stamp = ros::Time::now()-ros::Duration(1.0);
			xm_ik_client.call(srv);//将要转动的数据发给电机运算类，由其进行解算，其后将各个关节的数据发给此类，，由此类发给hardware
			if(!srv.response.result)
			{
				ROS_ERROR("IK failed!");//TODO add feedback
				res.result = false;
				return true;
			}
			res.result =true;
			
			std::vector<double> joint_value(5);
			joint_value = srv.response.solution;
		
			execute(joint_value);
			gripper_exe(false);

	       
		}
		if(req.action ==2)//place
		{
			srv.request.goal = req.goal_position;
			srv.request.goal.header.stamp = ros::Time::now()-ros::Duration(1.0);
			xm_ik_client.call(srv);
			if(!srv.response.result)
			{
				ROS_ERROR("IK failed!");//TODO add feedback
				res.result = false;
				return true;
			}
			std::vector<double> joint_value(5);
			joint_value = srv.response.solution;
			execute(joint_value);
			gripper_exe(true);
		}
		// display_plan();
		return true;
	}
	
	bool prepare_nav()
	{
		moveit::planning_interface::MoveGroup xm_arm_("xm_arm");
		xm_arm_.setNamedTarget("nav_pose");
		ROS_INFO("fuck");
		bool success = xm_arm_.plan(my_plan);
		if (!success)
			return false;
		ROS_INFO("Go to the pose using when executing the nav stack\n");
		bool hehe = false;
		while (hehe ==false)
		{
			hehe = xm_arm_.move();
		}
		
		return true;
	}

	

	// 机械臂执行函数
	bool execute(const std::vector<double> &goal)//向hardware发送数据
	{
		if(goal.size()!=5)
		{
			ROS_ERROR("Can not execute the num of goal is not 5");
			return false;
		}
		for(int i=0 ; i<5 ;i++) std::cout<<"Execute"<<goal[i]<<" ";
		std::cout<<std::endl;
		// 执行目标关节值
		moveit::planning_interface::MoveGroup xm_arm_("xm_arm");
		xm_arm_.setPoseReferenceFrame("base_footprint");
		
		ROS_INFO("Reference frame is: %s", xm_arm_.getPlanningFrame().c_str());

		ROS_INFO("endeffector frame is: %s", xm_arm_.getEndEffectorLink().c_str());	
		xm_arm_.setStartState(*xm_arm_.getCurrentState());	
		xm_arm_.setJointValueTarget(goal);
		moveit::planning_interface::MoveItErrorCode res;
		bool success = xm_arm_.plan(my_plan);
		ROS_WARN(" plan arm_goal is %s",success?"SUCCESS":"FAILED");
		if (!success)
			return false;
		bool hehe =false;
		while (hehe ==false)
		{
			hehe =xm_arm_.move();	
			std::cout<<hehe<<std::endl;
		}
		sleep(1.0);
		return true;
	}
	
	// 机械爪执行函数
	// true is open
	// false is close
	bool gripper_exe(bool mod)
	{
		std::vector<double> gripper_pose;
		gripper_pose.resize(2,0);
		
		
		if(mod ==true)
		{
			gripper_pose[0] = 0.0;
			gripper_pose[1] = 0.0;	
		}
		if(mod ==false)
		{
			gripper_pose[0] = -0.03;
			gripper_pose[1] = -0.03;	
		}
		moveit::planning_interface::MoveGroup xm_gripper_("xm_gripper");
		xm_gripper_.setJointValueTarget(gripper_pose);
		bool success = xm_gripper_.plan(my_plan);
		ROS_INFO("xm_gripper execute is ", success?"SUCCESS":"FAILED");
		if (!success)
			return false;
		xm_gripper_.move();	
		return true;
	}

	void display_plan()
	{
	ROS_INFO("Visualizing plan hahaha------");    
    display_trajectory.trajectory_start = my_plan.start_state_;
    display_trajectory.trajectory.push_back(my_plan.trajectory_);
    display_publisher.publish(display_trajectory);
    /* Sleep to give Rviz time to visualize the plan. */
    sleep(1.0);
	}

	
	ros::NodeHandle nh;
   	std::vector<double> init_value;
   	std::vector<double> current_cmd;
	ros::ServiceClient xm_ik_client;
	ros::ServiceServer arm_server;
	ros::Publisher display_publisher;
	moveit::planning_interface::MoveGroup::Plan my_plan;
	moveit_msgs::DisplayTrajectory display_trajectory;//显示轨迹??
};


int main(int argc,char **argv)
{
    ros::init(argc,argv,"xm_pick_or_place");
    ros::AsyncSpinner spiner(2);
    spiner.start();
	
    ros::NodeHandle n;
	
    xm_pick_or_place server(n);
    while(ros::ok());
    return 0;
}
