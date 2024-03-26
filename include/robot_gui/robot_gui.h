#pragma once

#define CVUI_IMPLEMENTATION
#include "robot_gui/cvui.h"
#include <robotinfo_msgs/RobotInfo10Fields.h>
#include <geometry_msgs/Twist.h>
#include "nav_msgs/Odometry.h"
#include <std_srvs/Trigger.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

class RobotGUI {
    public:
        RobotGUI();
        void run();

    private:
        ros::Subscriber info_sub_;
        ros::Subscriber odom_sub_;
        ros::Publisher twist_pub_;
        robotinfo_msgs::RobotInfo10Fields robot_data;
        std::string info_topic_name;
        std::string twist_topic_name;
        geometry_msgs::Twist twist_msg;
        float linear_velocity_step = 0.1;
        float angular_velocity_step = 0.1;
        void infoCallback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg);
        nav_msgs::Odometry odom_data;
        std::string odom_topic_name;
        void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
        ros::ServiceClient service_client;
        std_srvs::Trigger srv_req;
        std::string service_name;
        std::string last_service_call_msg;
        int service_call_counter = 0;
        const std::string WINDOW_NAME = "ROBOT GUI";
};