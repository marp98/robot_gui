#include "robot_gui/robot_gui.h"

RobotGUI::RobotGUI() {
  ros::NodeHandle nh;
  topic_name = "robot_info";
  sub_ = nh.subscribe<robotinfo_msgs::RobotInfo10Fields>(topic_name, 2,
                                         &RobotGUI::infoCallback, this);
  twist_topic_name = "cmd_vel";
  twist_pub_ = nh.advertise<geometry_msgs::Twist>(twist_topic_name, 10);
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

    int button_y = 220;
    if (cvui::button(frame, 160, button_y, 130, 60, " Forward ")) {
        twist_msg.linear.x = twist_msg.linear.x + linear_velocity_step;
        twist_pub_.publish(twist_msg);
    }

    button_y += 65; 
    if (cvui::button(frame, 25, button_y, 130, 60, " Left ")) {
        twist_msg.angular.z = twist_msg.angular.z + angular_velocity_step;
        twist_pub_.publish(twist_msg);
    }

    if (cvui::button(frame, 160, button_y, 130, 60, "   Stop  ")) {
        twist_msg.linear.x = 0.0;
        twist_msg.angular.z = 0.0;
        twist_pub_.publish(twist_msg);
    }

    if (cvui::button(frame, 295, button_y, 130, 60, " Right ")) {
        twist_msg.angular.z = twist_msg.angular.z - angular_velocity_step;
        twist_pub_.publish(twist_msg);
    }

    button_y += 65; 
    if (cvui::button(frame, 160, button_y, 130, 60, "Backward")) {
        twist_msg.linear.x = twist_msg.linear.x - linear_velocity_step;
        twist_pub_.publish(twist_msg);
    }

    int velocity_y = 420;
    cvui::window(frame, 10, velocity_y, 210, 40, "Linear velocity:");
    cvui::printf(frame, 35, velocity_y + 25, 0.4, 0xff0000, "%.02f m/sec", twist_msg.linear.x);

    cvui::window(frame, 230, velocity_y, 210, 40, "Angular velocity:");
    cvui::printf(frame, 255, velocity_y + 25, 0.4, 0xff0000, "%.02f rad/sec", twist_msg.angular.z);

    cvui::update();

    cv::imshow(WINDOW_NAME, frame);

    if (cv::waitKey(20) == 27) {
      break;
    }
    ros::spinOnce();
  }
}
