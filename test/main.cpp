#include "WebotsRobot.h"

int main(int argc,char** argv)
{
    ros::init(argc, argv, "webots_robot");
    WebotsRobot WebotsRobot;
    WebotsRobot.InitMotor();
    WebotsRobot.InitLidar();
    // WebotsRobot.InitGPS();
    WebotsRobot.InitIMU();
    //WebotsRobot.InitCamera();
    WebotsRobot.Loop();
    return 0;
}

