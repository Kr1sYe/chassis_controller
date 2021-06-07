#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <algorithm>
#include <string>

#include <math.h>
#include <stdlib.h>
#include <sys/poll.h>

#include <boost/thread/thread.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include <sstream>

#define KEYCODE_R 0x44
#define KEYCODE_L 0x41
#define KEYCODE_U 0x57
#define KEYCODE_D 0x58
#define KEYCODE_S 0x53
#define KEYCODE_ROTATE_L 0x51
#define KEYCODE_ROTATE_R 0x45
#define KEYCODE_G 0x47
#define KEYCODE_H 0x48
#define KEYCODE_J 0x4A
#define KEYCODE_Y 0x59
#define KEYCODE_N 0x4E

int kfd = 0;
struct termios cooked, raw;



void quit(int sig)
{
    (void)sig;

    tcsetattr(kfd, TCSANOW, &cooked);
    ros::shutdown();
    exit(0);
}

// class ChassisKeyboardThread
// {
//     public:
//         ChassisKeyboardThread();
//         ~ChassisKeyboardThread();
    
// };

// ChassisKeyboardThread::ChassisKeyboardThread()
// {
//     std::cout << "Construct ChassisKeyboardThread" << std::endl;
// }

// ChassisKeyboardThread::~ChassisKeyboardThread()
// {
//     std::cout << "Destruct ChassisKeyboardThread" << std::endl;
// }


class ChassisKeyboardNode
{
    public:
        ChassisKeyboardNode();
        ~ChassisKeyboardNode();

        void keyLoop();

        void pubmsg()
        {
            ros::Rate loop_rate(30);
            while (ros::ok())
            {
                ros::spinOnce();
                // ROS_INFO("pubmsg!!!");
                pub_keyboard_.publish(cmdvel_);

                loop_rate.sleep();
            }

        }

        void stopRobot()
        {

            cmdvel_.linear.x = 0.0;
            cmdvel_.linear.y = 0.0;
            cmdvel_.linear.z = 0.0;
            cmdvel_.angular.x = 0.0;
            cmdvel_.angular.y = 0.0;
            cmdvel_.angular.z = 0.0;
            pub_keyboard_.publish(cmdvel_);
        }

        void checkLinearVelLimit_X()
        {
            //  Limit Velocity X
            if(target_linear_x > MAX_LIN_VEL_X)
            {
                target_linear_x = MAX_LIN_VEL_X;
            }
            else if(target_linear_x < -MAX_LIN_VEL_X)
            {
                target_linear_x = -MAX_LIN_VEL_X;
            }
            else
            {
                target_linear_x = target_linear_x;
            }
        }

        void checkLinearVelLimit_Y()
        {
            //  Limit Velocity Y
            if(target_linear_y > MAX_LIN_VEL_Y)
            {
                target_linear_y = MAX_LIN_VEL_Y;
            }
            else if(target_linear_y < -MAX_LIN_VEL_Y)
            {
                target_linear_y = -MAX_LIN_VEL_Y;
            }
            else
            {
                target_linear_y = target_linear_y;
            }
        }

        void checkAngularVelLimit_Z()
        {
            //  Limit Angular Z
            if(target_angular_z > MAX_ANG_VEL_Z)
            {
                target_angular_z = MAX_ANG_VEL_Z;
            }
            else if(target_angular_z < -MAX_ANG_VEL_Z)
            {
                target_angular_z = -MAX_ANG_VEL_Z;
            }
            else
            {
                target_angular_z = target_angular_z;
            }
        }

        void makeSimplePlan()
        {   

            //-----linear x-----
            if (target_linear_x > control_linear_x)
            {
                control_linear_x = std::min(target_linear_x, control_linear_x +  LIN_VEL_STEP_SIZE_);
                ROS_INFO("control_angular_x: %f",control_linear_x);
            }
            else if (target_linear_x < control_linear_x)
            {
                control_linear_x = std::max(target_linear_x, control_linear_x -  LIN_VEL_STEP_SIZE_);
                ROS_INFO("control_angular_x: %f",control_linear_x);
            }
            else
            {
                control_linear_x = control_linear_x;
            }

            //-----linear y-----
            if (target_linear_y > control_linear_y)
            {
                control_linear_y = std::min(target_linear_y, control_linear_y +  LIN_VEL_STEP_SIZE_);
                ROS_INFO("control_angular_y: %f",control_linear_y);
            }
            else if (target_linear_y < control_linear_y)
            {
                control_linear_y = std::max(target_linear_y, control_linear_y -  LIN_VEL_STEP_SIZE_);
                ROS_INFO("control_angular_y: %f",control_linear_y);
            }
            else
            {
                control_linear_y = control_linear_y;
            }

            //-----angular z-----
            if (target_angular_z > control_angular_z)
            {
                control_angular_z = std::min(target_angular_z, control_angular_z +  ANG_VEL_STEP_SIZE_);
                ROS_INFO("control_angular_z: %f",control_angular_z);
            }
            else if (target_angular_z < control_angular_z)
            {
                control_angular_z = std::max(target_angular_z, control_angular_z -  ANG_VEL_STEP_SIZE_);
                ROS_INFO("control_angular_z: %f",control_angular_z);
            }
            else
            {
                control_angular_z = control_angular_z;
            }

            sleep(0.5);
        }

        



    private:
        
        ros::NodeHandle n_;
        //double linear_, angular_, l_scale_, a_scale_;
        ros::Publisher pub_keyboard_;
        ros::Publisher pub_keyboard_msg_;

        double target_linear_x, target_linear_y, target_linear_z;
        double target_angular_x, target_angular_y, target_angular_z;

        double control_linear_x, control_linear_y, control_linear_z;
        double control_angular_x, control_angular_y, control_angular_z;

        double l_scale_, a_scale_;

        std::string SET_CMD_VEL_;
        std::string SET_ARMCOMMAND_;
        double LIN_VEL_STEP_SIZE_, ANG_VEL_STEP_SIZE_;
        double MAX_LIN_VEL_X, MAX_LIN_VEL_Y, MAX_ANG_VEL_Z;
        
        geometry_msgs::Twist cmdvel_;
        std_msgs::String arm_command_msg_;

        boost::thread* chassis_keyboard_thread_;
        boost::thread* chassis_pub_thread_;
        
};

ChassisKeyboardNode::ChassisKeyboardNode():
                                        target_linear_x(0), target_linear_y(0), target_linear_z(0),
                                        target_angular_x(0), target_angular_y(0), target_angular_z(0),
                                        control_linear_x(0), control_linear_y(0), control_linear_z(0),
                                        control_angular_x(0), control_angular_y(0), control_angular_z(0)
                                        
    
{
    // nh_.param("scale_angular", a_scale_, a_scale_);
    // nh_.param("scale_linear", l_scale_, l_scale_);
    ROS_INFO("RUN Chassis_Keyboard");

    ros::NodeHandle n_private("~");

    n_private.param<double>("LIN_VEL_STEP_SIZE", LIN_VEL_STEP_SIZE_, 0.01);
    n_private.param<double>("ANG_VEL_STEP_SIZE", ANG_VEL_STEP_SIZE_, 0.01);
    n_private.param<double>("MAX_LIN_VEL_X", MAX_LIN_VEL_X, 0.1);
    n_private.param<double>("MAX_LIN_VEL_Y", MAX_LIN_VEL_Y, 0.1);
    n_private.param<double>("MAX_ANG_VEL_Z", MAX_ANG_VEL_Z, 0.2);
    n_private.param<std::string>("SET_CMD_VEL", SET_CMD_VEL_, "cmd_vel");
    n_private.param<std::string>("SET_ARMCOMMAND", SET_ARMCOMMAND_, "arm_command_msg");

    // pub_keyboard_ = n_.advertise<std_msgs::String>("keyboard_vel", 1);
    // pub_keyboard_ = n_.advertise<geometry_msgs::Twist>("/smooth_cmd_vel", 1);
    pub_keyboard_ = n_.advertise<geometry_msgs::Twist>(SET_CMD_VEL_, 1);
    pub_keyboard_msg_ = n_.advertise<std_msgs::String>(SET_ARMCOMMAND_, 1);

    chassis_keyboard_thread_ = new boost::thread(boost::bind(&ChassisKeyboardNode::keyLoop, this));
    // chassis_pub_thread_ = new boost::thread(boost::bind(&ChassisKeyboardNode::pubmsg, this));
    
    chassis_keyboard_thread_->join();
    // chassis_pub_thread_->join();

    stopRobot();



}

ChassisKeyboardNode::~ChassisKeyboardNode()
{ 
    ROS_INFO("Desstruction");
    stopRobot();
    chassis_keyboard_thread_->interrupt();
    delete chassis_keyboard_thread_;

    chassis_pub_thread_->interrupt();
    delete chassis_pub_thread_;

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "ChassisKeyboardNode");
    ChassisKeyboardNode chassis_teleop;


    // chassis_teleop.keyLoop();

    // chassis_teleop.stopRobot();
    
    return(0);
}


void ChassisKeyboardNode::keyLoop()
{
    signal(SIGINT,quit);
    
    char c;
    bool dirty = false;


    // get the console in raw mode                                                              
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file                         
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move the Chasis.");

    ros::Rate loop_rate(30);

    while (ros::ok())
    {
        ros::spinOnce();

        //get the next event from the keyboard  
        if(read(kfd, &c, 1) < 0)
        {
            perror("read():");
            exit(-1);
        }

        // linear_=angular_=0;
        ROS_DEBUG("value: 0x%02X\n", c);

        //-------
        std::cout << c <<std::endl;

        switch(c)
        {
            case KEYCODE_U:
            ROS_DEBUG("UP");
            target_linear_x = target_linear_x + LIN_VEL_STEP_SIZE_;
            checkLinearVelLimit_X();
            dirty = true;
            break;

            case KEYCODE_D:
            ROS_DEBUG("DOWN");
            target_linear_x = target_linear_x - LIN_VEL_STEP_SIZE_;
            checkLinearVelLimit_X();
            // ROS_INFO("RIGHT");
            dirty = true;
            break;

            case KEYCODE_L:
            ROS_DEBUG("LEFT");
             target_linear_y = target_linear_y + LIN_VEL_STEP_SIZE_;
            checkLinearVelLimit_Y();
            dirty = true;
            break;

            case KEYCODE_R:
            ROS_DEBUG("RIGHT");
            target_linear_y = target_linear_y - LIN_VEL_STEP_SIZE_;
            checkLinearVelLimit_Y();
            //linear_ = -1.0;
            dirty = true;
            break;

            case KEYCODE_S:
            ROS_DEBUG("STOP");
            target_linear_x = 0;
            target_linear_y = 0;
            target_linear_z = 0;
            target_angular_x = 0;
            target_angular_y = 0;
            target_angular_z = 0;
            dirty = true;
            break;

            case KEYCODE_ROTATE_L:
            ROS_DEBUG("ROTATE L");
            target_angular_z = target_angular_z + ANG_VEL_STEP_SIZE_;
            checkAngularVelLimit_Z();
            dirty = true;
            break;
                                                                                                                        
                                        
            case KEYCODE_ROTATE_R:
            ROS_DEBUG("ROTATE R");
            target_angular_z = target_angular_z - ANG_VEL_STEP_SIZE_;
            checkAngularVelLimit_Z();
            dirty = true;
            break;

            case KEYCODE_G:
            ROS_DEBUG("ROTATE G");
            arm_command_msg_.data = "turn left";
            pub_keyboard_msg_.publish(arm_command_msg_); 
            break;

            case KEYCODE_H:
            ROS_DEBUG("ROTATE H");
            arm_command_msg_.data = "arm_ready_start";
            pub_keyboard_msg_.publish(arm_command_msg_); 
            break;

            case KEYCODE_J:
            ROS_DEBUG("ROTATE J");
            arm_command_msg_.data = "turn right";
            pub_keyboard_msg_.publish(arm_command_msg_); 
            break;

            case KEYCODE_Y:
            ROS_DEBUG("ROTATE Y");
            arm_command_msg_.data = "move up";
            pub_keyboard_msg_.publish(arm_command_msg_); 
            break;

            case KEYCODE_N:
            ROS_DEBUG("ROTATE N");
            arm_command_msg_.data = "move down";
            pub_keyboard_msg_.publish(arm_command_msg_); 
            break;
        }
        ROS_INFO("Vel_X: %f, Vel_Y: %f, target_angular_z: %f", control_linear_x, control_linear_y, control_angular_z);
        
        makeSimplePlan();
  
        // switch(c)
        // {
        //     case 0x7b:
        //     ROS_DEBUG("ZUOKUO");
        //     ROS_INFO("ZUOKUO");
        //     break;

        //     case 0x47:
        //     ROS_DEBUG("G");
        //     ROS_INFO("G");
        //     break;

        //     case 0x54:
        //     ROS_DEBUG("T");
        //     ROS_INFO("T");
        //     break;

        //     case 0x45:
        //     ROS_DEBUG("E");
        //     ROS_INFO("E");
        //     break;

        //     case 0x4e:
        //     ROS_DEBUG("N");
        //     ROS_INFO("N");
        //     break;

        //     case 0x44:
        //     ROS_DEBUG("D");
        //     ROS_INFO("D");
        //     break;
        // }


        // geometry_msgs::Twist twist;
        // twist.angular.z = a_scale_*angular_;
        // twist.linear.x = l_scale_*linear_;
        cmdvel_.linear.x = control_linear_x;
        cmdvel_.linear.y = control_linear_y;
        cmdvel_.angular.z = control_angular_z;

        if(dirty ==true)
        {
            pub_keyboard_.publish(cmdvel_);   
            dirty=false;
        }

        arm_command_msg_.data = "";
        loop_rate.sleep();

    }


    return;
}
