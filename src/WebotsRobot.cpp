#include "WebotsRobot.h"

bool WebotsRobot::InitMotor()
{
    //初始化电机
    //step1 获取电机控制器名字
    ros::Subscriber name_sub = nh.subscribe("model_name", 100, &WebotsRobot::ControllerNameCallback,this);
    while (m_controller_count == 0 || m_controller_count < name_sub.getNumPublishers()) {
        ros::spinOnce();
        ros::spinOnce();
        ros::spinOnce();
    }
    ros::spinOnce();

    if (m_controller_count == 1)
        m_controller_name = m_controller_list[0];
    else {
        int wanted_controller = 0;
        std::cout << "Choose the # of the controller you want to use:\n";
        std::cin >> wanted_controller;
        if (1 <= wanted_controller && wanted_controller <= m_controller_count)
            m_controller_name = m_controller_list[wanted_controller - 1];
        else {
            ROS_ERROR("Invalid number for controller choice.");
            return false;
        }
    }
    ROS_INFO("Using controller: '%s'", m_controller_name.c_str());
    name_sub.shutdown();

    //step2 初始化电机位置与速度
    for (int i = 0; i < NMOTORS; ++i) {
        // position
        ros::ServiceClient set_position_client;
        webots_robot::set_float set_position_srv;
        set_position_client = nh.serviceClient<webots_robot::set_float>(std::string("pioneer3at/") + std::string(m_motor_name_list[i]) +
                                                                    std::string("/set_position"));

        set_position_srv.request.value = INFINITY;
        if (set_position_client.call(set_position_srv) && set_position_srv.response.success)
            ROS_INFO("Position set to INFINITY for motor %s.", m_motor_name_list[i].c_str());
        else
        {
            ROS_ERROR("Failed to call service set_position on motor %s.", m_motor_name_list[i].c_str());
            return false;
        }

        // speed
        ros::ServiceClient set_velocity_client;
        webots_robot::set_float set_velocity_srv;
        set_velocity_client = nh.serviceClient<webots_robot::set_float>(std::string("pioneer3at/") + std::string(m_motor_name_list[i]) +
                                                                    std::string("/set_velocity"));

        set_velocity_srv.request.value = 0.0;
        if (set_velocity_client.call(set_velocity_srv) && set_velocity_srv.response.success == 1)
            ROS_INFO("Velocity set to 0.0 for motor %s.", m_motor_name_list[i].c_str());
        else
        {
            ROS_ERROR("Failed to call service set_velocity on motor %s.", m_motor_name_list[i].c_str());
            return false;
        }
    }
    
    //step3 初始化里程计
    m_odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1);
    m_odom_timer = nh.createTimer(ros::Duration(0.01), boost::bind(&WebotsRobot::OdomPubCallback,this));

    //step4 开启速度控制话题
    m_vel_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &WebotsRobot::VelSubCallback,this);
    return true;
}

bool WebotsRobot::InitLidar()
{
    //初始化激光雷达
    ros::ServiceClient set_lidar_client;
    webots_robot::set_int lidar_srv;
    ros::Subscriber sub_lidar_scan;

    set_lidar_client = nh.serviceClient<webots_robot::set_int>("pioneer3at/Sick_LMS_291/enable");
    lidar_srv.request.value = TIME_STEP;
    if (set_lidar_client.call(lidar_srv) && lidar_srv.response.success) {
        ROS_INFO("Lidar enabled.");
    } else {
        if (!lidar_srv.response.success)
            ROS_ERROR("Sampling period is not valid.");
        ROS_ERROR("Failed to enable lidar.");
        return false;
    }
    return true;
}

bool WebotsRobot::InitGPS()
{
    //初始化GPS
    ros::ServiceClient set_GPS_client;
    webots_robot::set_int GPS_srv;
    ros::Subscriber sub_GPS;
    set_GPS_client = nh.serviceClient<webots_robot::set_int>("pioneer3at/gps/enable");
    GPS_srv.request.value = 32;
    if (set_GPS_client.call(GPS_srv) && GPS_srv.response.success) {
        ROS_INFO("GPS enabled.");
    } else {
        if (!GPS_srv.response.success)
            ROS_ERROR("Sampling period is not valid.");
        ROS_ERROR("Failed to enable GPS.");
        return false;
    }
    return true;
}

bool WebotsRobot::InitIMU()
{
    //初始化姿态传感器
    ros::ServiceClient set_inertial_unit_client;
    webots_robot::set_int inertial_unit_srv;
    ros::Subscriber sub_inertial_unit;
    set_inertial_unit_client = nh.serviceClient<webots_robot::set_int>("pioneer3at/inertial_unit/enable");
    inertial_unit_srv.request.value = 32;
    if (set_inertial_unit_client.call(inertial_unit_srv) && inertial_unit_srv.response.success) {
        ROS_INFO("Inertial unit enabled.");
    } else {
        if (!inertial_unit_srv.response.success)
            ROS_ERROR("Sampling period is not valid.");
        ROS_ERROR("Failed to enable inertial unit.");
        return false;
    }

    //初始化角速度传感器
    ros::ServiceClient set_gyro_client;
    webots_robot::set_int gyro_srv;
    ros::Subscriber sub_gyro;
    set_gyro_client = nh.serviceClient<webots_robot::set_int>("pioneer3at/gyro/enable");
    gyro_srv.request.value = 32;
    if (set_gyro_client.call(gyro_srv) && gyro_srv.response.success) {
        ROS_INFO("Gyro enabled.");
    } else {
        if (!gyro_srv.response.success)
            ROS_ERROR("Sampling period is not valid.");
        ROS_ERROR("Failed to enable Gyro.");
        return false;
    }

    //初始化线速度传感器
    ros::ServiceClient set_accelerometer_client;
    webots_robot::set_int accelerometer_srv;
    ros::Subscriber sub_accelerometer;
    set_accelerometer_client = nh.serviceClient<webots_robot::set_int>("pioneer3at/accelerometer/enable");
    accelerometer_srv.request.value = 32;
    if (set_accelerometer_client.call(accelerometer_srv) && accelerometer_srv.response.success) {
        ROS_INFO("Accelerometer enabled.");
    } else {
        if (!accelerometer_srv.response.success)
            ROS_ERROR("Sampling period is not valid.");
        ROS_ERROR("Failed to enable Accelerometer.");
        return false;
    }
    return true;
}

bool WebotsRobot::InitCamera()
{
    //初始化相机
    ros::ServiceClient set_camera_client;
    webots_robot::set_int camera_srv;
    ros::Subscriber sub_camera;
    set_camera_client = nh.serviceClient<webots_robot::set_int>("pioneer3at/camera/enable");
    camera_srv.request.value = 64;
    if (set_camera_client.call(camera_srv) && camera_srv.response.success) {
        ROS_INFO("Camera enabled.");
    } else {
        if (!camera_srv.response.success)
            ROS_ERROR("Sampling period is not valid.");
        ROS_ERROR("Failed to enable Camera.");
        return false;
    }
}

void WebotsRobot::Loop()
{
    // 还需要弄清楚这里是干嘛的
    // webots_robot::set_int timestep_srv;
    // timestep_srv.request.value = TIME_STEP;
    // while (ros::ok()) {
    //     if (!timestep_client.call(timestep_srv) || !timestep_srv.response.success) {
    //         ROS_ERROR("Failed to call service time_step for next step.");
    //         break;
    //         ros::spinOnce();
    //     }
    //     ros::spinOnce();
    // }

    // timestep_srv.request.value = 0;
    // timestep_client.call(timestep_srv);
    ros::spin();
    ros::shutdown();
}

void WebotsRobot::ControllerNameCallback(const std_msgs::String::ConstPtr &name)
{
    m_controller_count++;
    m_controller_list.push_back(name->data);
    ROS_INFO("Controller #%d: %s.", m_controller_count, m_controller_list.back().c_str());
    if (m_controller_count == 1)
        m_controller_name = m_controller_list[0];
    else {
        int wanted_controller = 0;
        std::cout << "Choose the # of the controller you want to use:\n";
        std::cin >> wanted_controller;
        if (1 <= wanted_controller && wanted_controller <= m_controller_count)
            m_controller_name = m_controller_list[wanted_controller - 1];
        else {
            ROS_ERROR("Invalid number for controller choice.");
            return ;
        }
    }
}

void WebotsRobot::OdomPubCallback()
{
    for (int i = 0; i < NMOTORS; ++i) {
        // speed
        ros::ServiceClient get_velocity_client;
        webots_robot::get_float get_velocity_srv;
        get_velocity_client = nh.serviceClient<webots_robot::get_float>(std::string("pioneer3at/") + std::string(m_motor_name_list[i]) +
                                                                    std::string("/get_velocity"));
        get_velocity_srv.request.ask = true;
        if (get_velocity_client.call(get_velocity_srv))
            ROS_INFO("Velocity is %f , get from motor %s.", get_velocity_srv.response.value , m_motor_name_list[i].c_str());
        else
        {
            ROS_ERROR("Failed to call service get_velocity on motor %s.", m_motor_name_list[i].c_str());
            return ;
        }
    }
}

void WebotsRobot::VelSubCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    //std::cout << msg->linear.x <<std::endl;
    for (int i = 0; i < NMOTORS; ++i) {
        // speed
        ros::ServiceClient set_velocity_client;
        webots_robot::set_float set_velocity_srv;
        set_velocity_client = nh.serviceClient<webots_robot::set_float>(std::string("pioneer3at/") + std::string(m_motor_name_list[i]) +
                                                                    std::string("/set_velocity"));

        set_velocity_srv.request.value = msg->linear.x;
        if (set_velocity_client.call(set_velocity_srv) && set_velocity_srv.response.success == 1)
            ROS_INFO("Velocity set to %f for motor %s.", set_velocity_srv.request.value , m_motor_name_list[i].c_str());
        else
        {
            ROS_ERROR("Failed to call service set_velocity on motor %s.", m_motor_name_list[i].c_str());
            return ;
        }
    }

}