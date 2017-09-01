#include <xm_arm_nav/xm_keyboard_teleop_arm.h>

int kfd = 0;
// mystical structure
struct termios cooked, raw;

XMKeyboardTeleopNode::XMKeyboardTeleopNode()
{   
    ros::NodeHandle xm_nh_("/");
    arm_pos_client_ = xm_nh_.serviceClient<xm_msgs::xm_PickOrPlace>("moveit_planning");
    arm_pos_client_.waitForExistence(ros::Duration(30));
}

XMKeyboardTeleopNode::~XMKeyboardTeleopNode()
{

}

void XMKeyboardTeleopNode::StopArm()
{
    ROS_WARN("byebye for use ^_^");
}

void XMKeyboardTeleopNode::KeyboardLoop()
{
    char keyboard_cmd;
    bool dirty = false;
    geometry_msgs::Point joint_pos_;
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("Use WASDQE keys to control xm's Arm");

    struct pollfd ufd;
    ufd.fd = kfd;
    ufd.events = POLLIN;
    ros::Rate loop(10);

    while (xm_nh_.ok())
    {
        boost::this_thread::interruption_point();
        // get the next event from the keyboard
        int num;
        
        if((num = poll(&ufd, 1, 250)) < 0)
        {
            perror("poll():");
            return;
        }
        else if(num > 0)
        {
            if(read(kfd, &keyboard_cmd, 1) < 0)
            {
                perror("read():");
                return;
            }
        }
        else
        {
            if (dirty == false)
                std::cout<<"please type the keys"<<std::endl;
                continue;
            if (dirty  ==true)
                dirty =false;
        }
        ROS_ERROR("fuck\n");
        std::cout<<dirty<<std::endl;
        std::cout<<num<<std::endl;
        
        tf::StampedTransform transform;
        try
        {
            ros::Time now = ros::Time(0); 
            listener.waitForTransform("base_link", "gripper_link",
                              now, ros::Duration(1.0));
            listener.lookupTransform("base_link", "gripper_link",  
                               ros::Time(0), transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        // we should change the current state of arm
        joint_pos_.x = transform.getOrigin().x();
        joint_pos_.y = transform.getOrigin().y();
        joint_pos_.z = transform.getOrigin().z();
        ROS_INFO_STREAM("the current state is "<<(joint_pos_.x)<<" "<<(joint_pos_.y)<<" "<<(joint_pos_.z));
        switch(keyboard_cmd)
        {
            case KEYCODE_Q:
                dirty = true;
                joint_pos_.z += 0.1;
                 
                {
                    ROS_WARN("  move Arm to high");
                }
                break;
            case KEYCODE_E:
                dirty = true;
                joint_pos_.z -= 0.1;
                 
                {
                    ROS_WARN("  move Arm to low");
                }
                break;
            case KEYCODE_A:
                dirty = true;
                joint_pos_.y += 0.1;
                 
                {
                    ROS_WARN("  move Arm to left");
                }
                break;
            case KEYCODE_D:
                dirty = true;
                joint_pos_.y -= 0.1;
                 
                {
                    ROS_WARN("  move Arm to right");
                }
                break;
            case KEYCODE_W:
                dirty = true;
                joint_pos_.x += 0.1;
                 
                {
                    ROS_WARN("  move Arm to front");
                }
                break;
            case KEYCODE_S:
                dirty = true;
                joint_pos_.x -= 0.1;
                 
                {
                    ROS_WARN(" move Arm to back");
                }
                break;
        }
        ros::Rate rate(1);
        rate.sleep();
        arm_pos_.request.action =1;
        arm_pos_.request.goal_position.header.frame_id ="base_link";
        arm_pos_.request.goal_position.point = joint_pos_;
        if (arm_pos_client_.call(arm_pos_))
        {
            ROS_WARN_STREAM("arm teleop result is"<<(arm_pos_.response.result)? "succeeded":"aborted");
        }
        else
        {
            ROS_ERROR("call service failed");
        }
        loop.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "xm_keyboard_teleop_arm", ros::init_options::NoSigintHandler);
    XMKeyboardTeleopNode tbk;

    boost::thread t = boost::thread(boost::bind(&XMKeyboardTeleopNode::KeyboardLoop, &tbk));

    ros::spin();

    t.interrupt();
    t.join();
    tbk.StopArm();
    tcsetattr(kfd, TCSANOW, &cooked);

    return(0);
}