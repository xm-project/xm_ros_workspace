#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <xm_msgs/xm_SolveIK.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <string.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PointStamped.h>
// 机械臂尺寸
const double l_big_arm = 0.35;
const double l_fore_arm = 0.25;
const double l_gripper = 0.25;
const double PI = 3.1415926;

//  joint_lifting ,joint_waist , joint_big_arm, Joint_fore_arm, Joint_wrist
// ik解算器，和scripts文件夹的python实现一样
const double joint_range_max[5] = {0.035, 1.57, 1.57, 2.10, 2.2};
const double joint_range_min[5] = { -0.18, -0.35, -1.57, -2.10, -2.2};
const double arm_max_len = 0.6;
const double arm_min_len = 0.3347312354710866;

class xm_arm_ik_solver
{
public:
	xm_arm_ik_solver(ros::NodeHandle &n)
		: nh(n), tf_(ros::Duration(5.0))
	{
		ik_server = nh.advertiseService("xm_ik_solver", &xm_arm_ik_solver::service_callback, this);
		error_message = "ik succeeded";
	}
	~xm_arm_ik_solver()
	{
	}
private:
	bool service_callback(xm_msgs::xm_SolveIKRequest &req, xm_msgs::xm_SolveIKResponse &res)
	{
		
		geometry_msgs::PointStamped goal;
		std::vector<double> value;
		value.resize(5, 0.0);
		bool result  = false;
        tf_.transformPoint("base_link", req.goal, goal);
		
		std::cout << "IK Goal" << goal.point.x << " " << goal.point.y << " " << goal.point.z << std::endl;
		result = compute_ik(goal, value);
		
        res.result = result;
		res.solution = value;
		res.message = error_message;
		return true;
	}
	bool compute_ik(const geometry_msgs::PointStamped &goal, std::vector<double> &joint_value)
	{
		joint_value.resize(5,0.0);
		boost::shared_ptr<const sensor_msgs::JointState> msgptr;
		msgptr = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");
        std::vector<double> current_joint_states;
		// for (int i=0;i<msgptr->position.size();i++)
		// 	std::cout<<msgptr->position[i]<<std::endl;
		for (int i=0;i<5;i++)
			current_joint_states.push_back(msgptr->position[i]);
		// for (int i=0;i<5;i++)
		// 	std::cout<<current_joint_states[i]<<std::endl;
		geometry_msgs::PointStamped arm_goal;
		arm_goal.point = goal.point;
		arm_goal.header.frame_id = goal.header.frame_id;
		geometry_msgs::PointStamped waist_ps;
		tf_.transformPoint("waist_link",arm_goal,waist_ps);
		double waist_angle = atan2(waist_ps.point.y, waist_ps.point.x);
		joint_value[1]= waist_angle + current_joint_states[1];
		waist_ps.point.x = waist_ps.point.x - l_gripper*cos(waist_angle);
		waist_ps.point.y = waist_ps.point.y - l_gripper*sin(waist_angle);

		tf::StampedTransform transform;
		try
		{
		tf_.lookupTransform("waist_link", "big_arm_link",  
								ros::Time(0), transform);
		}
		catch (tf::TransformException ex)
		{
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
		}
		geometry_msgs::Point target_state;
		target_state.x = waist_ps.point.x - transform.getOrigin().x();
		target_state.y = waist_ps.point.y - transform.getOrigin().y();
		target_state.z = waist_ps.point.z - transform.getOrigin().z();
		double temp_len;
		temp_len = sqrt(pow(target_state.x,2)+ pow(target_state.y,2));
		double out_len =0.130;
		double target_len_temp;
		// double target_len_temp = sqrt(pow(temp_len,2) - pow((out_len*sin(waist_angle)),2)) + out_len*cos(waist_angle);
		target_len_temp = temp_len;
		double target_len = sqrt(pow(target_len_temp,2)+pow(target_state.z,2));
		std::cout<<target_len<<std::endl;
		std::cout<<target_len_temp<<std::endl;
		if (target_len < arm_max_len && target_len >arm_min_len)
		{
			temp_len = target_state.z;
			joint_value[0]= current_joint_states[0];
		}
		else
		{
			if(target_len >arm_max_len)
			{
				ROS_WARN("the target is so far, I will move the lifting_link to catch it");
				std::cout<<arm_max_len<<std::endl;
				std::cout<<target_len_temp<<std::endl;
				if (arm_max_len < target_len_temp)
				{
					ROS_WARN("too far cannot receive with lifting_link");
					error_message = "too far cannot receive with lifting_link";
					return false;
				}
				temp_len = sqrt(pow(arm_max_len,2) - pow(target_len_temp,2));
				temp_len = (target_state.z>0)? temp_len:(-temp_len);
				double move_len = target_state.z - temp_len;
				joint_value[0]= move_len + current_joint_states[0];
			}
			else
			{
				ROS_WARN("the target is so close, I will move the lifting_link to catch it");
				temp_len = sqrt(pow(arm_min_len,2) - pow(target_len_temp,2));
				temp_len = (target_state.z>0)? temp_len:(-temp_len);
				double move_len = target_state.z - temp_len;
				joint_value[0]= move_len + current_joint_states[0];
			}
		}
		double distance = target_len_temp;
		double height = temp_len;
		double point_len = sqrt(pow(distance,2)+ pow(height,2));
		double acos_angle;
		acos_angle = (pow(l_big_arm,2) + pow(l_fore_arm,2) - pow(point_len,2))/(2*l_big_arm*l_fore_arm);
		acos_angle = acos_angle<-1? -1:acos_angle;
		joint_value[3] = acos(acos_angle) -PI;
		double joint_stick = atan2(height,distance);
		acos_angle = (pow(l_big_arm,2) + pow(point_len,2) - pow(l_fore_arm,2))/(2*l_big_arm*point_len);
		acos_angle = acos_angle>1? 1:acos_angle;
		joint_value[2] = joint_stick + acos(acos_angle);
		joint_value[4] = -(joint_value[2] + joint_value[3]);
		if (check(joint_value))
		{
			error_message ="ik succeeded";
			return true;
		}
		else
		{
			joint_value[3] = -joint_value[3];
			joint_value[2] = joint_stick*2 - joint_value[2];
			if (check(joint_value))
			{
				error_message ="ik succeeded";				
				return true;
			}
				
			else
			{
				ROS_WARN("joint_value not fit the limit");
				error_message = "joint_value not fit the limit";
				return false;
			}
		}

	} 	
	bool check(const std::vector<double> value)
	{
		
		std::cout<<"value checked are: ";
		for (int i = 0; i < value.size(); ++i)
				{
					std::cout << value[i] << " ";
				}
		std::cout << std::endl;
		for (int i = 0; i < value.size(); ++i)
		{
			if ((value[i] > joint_range_max[i]) || (value[i] < joint_range_min[i])  || (std::isnan(value[i]) ) )
			{
				ROS_ERROR("Value Not Fit !");
				std::cout << "Invalid value: ";
				for (int i = 0; i < value.size(); ++i)
				{
					std::cout << value[i] << " ";
				}
				std::cout << std::endl;
				ROS_INFO("check failed!");
				return false;
			}
		}
		ROS_INFO("check succeed !");
		return true;
	}
	ros::NodeHandle nh;
	ros::ServiceServer  ik_server;
	tf::TransformListener tf_;
	std::string error_message;
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "xm_ik_server");
	ros::NodeHandle n;
	xm_arm_ik_solver solver(n);
	ros::spin();
	return 0;
}
