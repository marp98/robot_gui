#include "robot_gui/robot_gui.h"
#include <algorithm>

RobotGUI::RobotGUI() {
  ros::NodeHandle nh;
  topic_name = "robot_info";
  sub_ = nh.subscribe<robotinfo_msgs::RobotInfo10Fields>(topic_name, 2,
                                         &RobotGUI::infoCallback, this);
}

void RobotGUI::infoCallback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg)
{
    robot_data = *msg;

    ROS_DEBUG("Robot data received:");
    ROS_DEBUG("  data_field_01: %s", msg->data_field_01.c_str());
    ROS_DEBUG("  data_field_02: %s", msg->data_field_02.c_str());
    ROS_DEBUG("  data_field_03: %s", msg->data_field_03.c_str());
    ROS_DEBUG("  data_field_04: %s", msg->data_field_04.c_str());
    ROS_DEBUG("  data_field_05: %s", msg->data_field_05.c_str());
    ROS_DEBUG("  data_field_06: %s", msg->data_field_06.c_str());
    ROS_DEBUG("  data_field_07: %s", msg->data_field_07.c_str());
    ROS_DEBUG("  data_field_08: %s", msg->data_field_08.c_str());
    ROS_DEBUG("  data_field_09: %s", msg->data_field_09.c_str());
    ROS_DEBUG("  data_field_10: %s", msg->data_field_10.c_str());
}

void RobotGUI::run() {
  cv::Mat frame = cv::Mat(600, 450, CV_8UC3);

  cv::namedWindow(WINDOW_NAME);
  cvui::init(WINDOW_NAME);

  while (ros::ok()) {
    frame = cv::Scalar(49, 52, 49);

    cvui::window(frame, 10, 20, 430, 190, "Info");

    cvui::printf(frame, 15, 45, 0.4, 0xffffff, robot_data.data_field_01.c_str());
    cvui::printf(frame, 15, 65, 0.4, 0xffffff, robot_data.data_field_02.c_str());
    cvui::printf(frame, 15, 85, 0.4, 0xffffff, robot_data.data_field_03.c_str());
    cvui::printf(frame, 15, 105, 0.4, 0xffffff, robot_data.data_field_04.c_str());
    cvui::printf(frame, 15, 125, 0.4, 0xffffff, robot_data.data_field_05.c_str());
    cvui::printf(frame, 15, 145, 0.4, 0xffffff, robot_data.data_field_06.c_str());
    cvui::printf(frame, 15, 165, 0.4, 0xffffff, robot_data.data_field_07.c_str());
    cvui::printf(frame, 15, 185, 0.4, 0xffffff, robot_data.data_field_08.c_str());

    cvui::update();

    cv::imshow(WINDOW_NAME, frame);

    if (cv::waitKey(20) == 27) {
      break;
    }
    ros::spinOnce();
  }
}
