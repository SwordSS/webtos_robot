#ifndef _WEBOTS_ROBOT_H_
#define _WEBOTS_ROBOT_H_

#include "ros/ros.h"
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include <webots_robot/set_float.h>
#include <webots_robot/set_int.h>
#include <webots_robot/get_float.h>

#include <vector>
#include <string>
#include <cmath>
#include <signal.h>


#define TIME_STEP 32
#define NMOTORS 4

class WebotsRobot
{
public:
    WebotsRobot():
    pn("~"),
    m_controller_count(0)
    {
        std::string temp_motor_name[NMOTORS] = {"front_left_wheel", "front_right_wheel", "back_left_wheel", "back_right_wheel"};
        std::vector<std::string> temp_motor_name_list(temp_motor_name,temp_motor_name+NMOTORS);
        m_motor_name_list = temp_motor_name_list;
        timestep_client = nh.serviceClient<webots_robot::set_int>("pioneer3at/robot/time_step");
        signal(SIGINT, Quit);
    };
    bool InitMotor();
    bool InitLidar();
    bool InitGPS();
    bool InitIMU();
    bool InitCamera();
    void Loop();
    

private:
    static void Quit(int sig) {
        ROS_INFO("User stopped the 'pioneer3at' node.");
        ros::NodeHandle nh;
        ros::ServiceClient temp_timestep_client = nh.serviceClient<webots_robot::set_int>("pioneer3at/robot/time_step");
        webots_robot::set_int timestep_srv;
        timestep_srv.request.value = 0;
        temp_timestep_client.call(timestep_srv);
        ros::shutdown();
        exit(0);
    }

    void ControllerNameCallback(const std_msgs::String::ConstPtr &name);
    void OdomPubCallback();
    void VelSubCallback(const geometry_msgs::Twist::ConstPtr& msg);

private:
    ros::ServiceClient timestep_client;
    ros::NodeHandle nh;
    ros::NodeHandle pn;
    int m_controller_count;
    std::string m_controller_name;
    std::vector<std::string> m_controller_list;
    std::vector<std::string> m_motor_name_list;

    ros::Timer m_odom_timer;
    ros::Publisher m_odom_pub;
    ros::Subscriber m_vel_sub;
};

#endif