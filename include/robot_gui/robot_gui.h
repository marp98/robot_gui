#pragma once

#define CVUI_IMPLEMENTATION
#include "robot_gui/cvui.h"
#include "std_msgs/Float64.h"
#include <robotinfo_msgs/RobotInfo10Fields.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

class RobotGUI {
    public:
        RobotGUI();
        void run();

    private:
        ros::Subscriber sub_;
        robotinfo_msgs::RobotInfo10Fields robot_data;
        std::string topic_name;
        void infoCallback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg);
        const std::string WINDOW_NAME = "ROBOT GUI";
};