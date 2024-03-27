#include "robot_gui/robot_gui.h"

RobotGUI::RobotGUI() {
  ros::NodeHandle nh;
  info_topic_name = "robot_info";
  info_sub_ = nh.subscribe<robotinfo_msgs::RobotInfo10Fields>(
      info_topic_name, 2, &RobotGUI::infoCallback, this);
  twist_topic_name = "cmd_vel";
  twist_pub_ = nh.advertise<geometry_msgs::Twist>(twist_topic_name, 10);

  odom_topic_name = "odom";
  odom_sub_ = nh.subscribe<nav_msgs::Odometry>(odom_topic_name, 2,
                                               &RobotGUI::odomCallback, this);

  service_client = nh.serviceClient<std_srvs::Trigger>("get_distance");
  service_name = "get_distance";
}

void RobotGUI::infoCallback(
    const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg) {
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

void RobotGUI::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  odom_data = *msg;
  ROS_DEBUG("Position x,y,z: [%0.2f, %0.2f, %0.2f]", msg->pose.pose.position.x,
            msg->pose.pose.position.y, msg->pose.pose.position.z);
}

void RobotGUI::run() {
  cv::Mat frame = cv::Mat(750, 450, CV_8UC3);

  cv::namedWindow(WINDOW_NAME);
  cvui::init(WINDOW_NAME);

  ros::Rate loop_rate(10);

  while (ros::ok()) {
    frame = cv::Scalar(49, 52, 49);

    cvui::window(frame, 10, 20, 430, 190, "Info");

    cvui::printf(frame, 15, 45, 0.4, 0xffffff,
                 robot_data.data_field_01.c_str());
    cvui::printf(frame, 15, 65, 0.4, 0xffffff,
                 robot_data.data_field_02.c_str());
    cvui::printf(frame, 15, 85, 0.4, 0xffffff,
                 robot_data.data_field_03.c_str());
    cvui::printf(frame, 15, 105, 0.4, 0xffffff,
                 robot_data.data_field_04.c_str());
    cvui::printf(frame, 15, 125, 0.4, 0xffffff,
                 robot_data.data_field_05.c_str());
    cvui::printf(frame, 15, 145, 0.4, 0xffffff,
                 robot_data.data_field_06.c_str());
    cvui::printf(frame, 15, 165, 0.4, 0xffffff,
                 robot_data.data_field_07.c_str());
    cvui::printf(frame, 15, 185, 0.4, 0xffffff,
                 robot_data.data_field_08.c_str());

    int button_y = 220;
    if (cvui::button(frame, 160, button_y, 130, 60, " Forward ")) {
      twist_msg.linear.x += linear_velocity_step;
    }

    button_y += 65;
    if (cvui::button(frame, 25, button_y, 130, 60, " Left ")) {
      twist_msg.angular.z += angular_velocity_step;
    }

    if (cvui::button(frame, 160, button_y, 130, 60, "   Stop  ")) {
      twist_msg.linear.x = 0.0;
      twist_msg.angular.z = 0.0;
    }

    if (cvui::button(frame, 295, button_y, 130, 60, " Right ")) {
      twist_msg.angular.z -= angular_velocity_step;
    }

    button_y += 65;
    if (cvui::button(frame, 160, button_y, 130, 60, "Backward")) {
      twist_msg.linear.x -= linear_velocity_step;
    }

    twist_pub_.publish(twist_msg);

    int velocity_y = 420;
    cvui::window(frame, 10, velocity_y, 210, 40, "Linear velocity:");
    cvui::printf(frame, 35, velocity_y + 25, 0.4, 0xff0000, "%.02f m/sec",
                 twist_msg.linear.x);

    cvui::window(frame, 230, velocity_y, 210, 40, "Angular velocity:");
    cvui::printf(frame, 255, velocity_y + 25, 0.4, 0xff0000, "%.02f rad/sec",
                 twist_msg.angular.z);

    cvui::text(frame, 25, 480, "Estimated robot position based off odometry");

    cvui::window(frame, 25, 495, 130, 90, "X");
    cvui::printf(frame, 25, 520, 0.4, 0xffffff, "%.02f",
                 odom_data.pose.pose.position.x);

    cvui::window(frame, 160, 495, 130, 90, "Y");
    cvui::printf(frame, 160, 520, 0.4, 0xffffff, "%.02f",
                 odom_data.pose.pose.position.y);

    cvui::window(frame, 295, 495, 130, 90, "Z");
    cvui::printf(frame, 295, 520, 0.4, 0xffffff, "%.02f",
                 odom_data.pose.pose.position.z);

    cvui::text(frame, 10, 605, "Distance travelled");

    if (cvui::button(frame, 10, 625, 100, 100, "Get distance")) {
      if (service_client.call(srv_req)) {
        ROS_DEBUG("Response message: %s", srv_req.response.message.c_str());
        last_service_call_msg = srv_req.response.message;
        service_call_counter++;
      } else {
        last_service_call_msg = "Service call failed.";
        service_call_counter = 0;
      }
    }

    cvui::window(frame, 120, 625, 320, 100, "Distance in meters:");

    if (not last_service_call_msg.empty()) {
      cvui::printf(frame, 120, 650, 0.4, 0xffffff, "%s",
                   last_service_call_msg.c_str());
    }    

    cvui::update();

    cv::imshow(WINDOW_NAME, frame);

    if (cv::waitKey(20) == 27) {
      break;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}
