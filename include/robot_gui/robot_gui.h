#pragma once

#define CVUI_IMPLEMENTATION
#include "robot_gui/cvui.h"
#include "std_msgs/Float64.h"
#include <robotinfo_msgs/RobotInfo10Fields.h>
#include <geometry_msgs/Twist.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

class RobotGUI {
    public:
        RobotGUI();
        void run();

    private:
        ros::Subscriber sub_;
        ros::Publisher twist_pub_;
        robotinfo_msgs::RobotInfo10Fields robot_data;
        std::string topic_name;
        std::string twist_topic_name;
        geometry_msgs::Twist twist_msg;
        float linear_velocity_step = 0.1;
        float angular_velocity_step = 0.1;
        void infoCallback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg);
        const std::string WINDOW_NAME = "ROBOT GUI";
};