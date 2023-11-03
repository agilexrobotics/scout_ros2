/*
 * scout_base_ros.cpp
 *
 * Created on: Oct 15, 2021 14:35
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#include "scout_base/scout_base_ros.hpp"

#include "scout_base/scout_messenger.hpp"
#include "ugv_sdk/utilities/protocol_detector.hpp"

namespace westonrobot {
ScoutBaseRos::ScoutBaseRos(std::string node_name)
    : rclcpp::Node(node_name), keep_running_(false) {
  this->declare_parameter("port_name", "can0");

  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("base_frame", "base_link");
  this->declare_parameter("odom_topic_name", "odom");

  this->declare_parameter("is_scout_mini", false);
  this->declare_parameter("is_omni_wheel", false);

  this->declare_parameter("simulated_robot", false);
  this->declare_parameter("control_rate", 50);

  LoadParameters();
}

void ScoutBaseRos::LoadParameters() {
  this->get_parameter_or<std::string>("port_name", port_name_, "can0");

  this->get_parameter_or<std::string>("odom_frame", odom_frame_, "odom");
  this->get_parameter_or<std::string>("base_frame", base_frame_, "base_link");
  this->get_parameter_or<std::string>("odom_topic_name", odom_topic_name_,
                                      "odom");

  this->get_parameter_or<bool>("is_scout_mini", is_scout_mini_, false);
  this->get_parameter_or<bool>("is_omni_wheel", is_omni_wheel_, false);

  this->get_parameter_or<bool>("simulated_robot", simulated_robot_, false);
  this->get_parameter_or<int>("control_rate", sim_control_rate_, 50);

  std::cout << "Loading parameters: " << std::endl;
  std::cout << "- port name: " << port_name_ << std::endl;
  std::cout << "- odom frame name: " << odom_frame_ << std::endl;
  std::cout << "- base frame name: " << base_frame_ << std::endl;
  std::cout << "- odom topic name: " << odom_topic_name_ << std::endl;

  std::cout << "- is scout mini: " << std::boolalpha << is_scout_mini_
            << std::endl;
  std::cout << "- is omni wheel: " << std::boolalpha << is_omni_wheel_
            << std::endl;

  std::cout << "- simulated robot: " << std::boolalpha << simulated_robot_
            << std::endl;
  std::cout << "- sim control rate: " << sim_control_rate_ << std::endl;
  std::cout << "----------------------------" << std::endl;
}

bool ScoutBaseRos::Initialize() {
  if (is_scout_mini_) {
    std::cout << "Robot base: Scout Mini" << std::endl;
  } else {
    std::cout << "Robot base: Scout" << std::endl;
  }

  ProtocolDetector detector;
  if (detector.Connect(port_name_)) {
    auto proto = detector.DetectProtocolVersion(5);
    if (proto == ProtocolVersion::AGX_V1) {
      std::cout << "Detected protocol: AGX_V1" << std::endl;
      if (!is_omni_wheel_) {
        is_omni_ = false;
        robot_ = std::make_shared<ScoutRobot>(ProtocolVersion::AGX_V1,
                                              is_scout_mini_);
        if (is_scout_mini_) {
          std::cout << "Creating interface for Scout Mini with AGX_V1 Protocol"
                    << std::endl;
        } else {
          std::cout << "Creating interface for Scout with AGX_V1 Protocol"
                    << std::endl;
        }
      } else {
        is_omni_ = true;
        omni_robot_ = std::unique_ptr<ScoutMiniOmniRobot>(
            new ScoutMiniOmniRobot(ProtocolVersion::AGX_V1));
        std::cout
            << "Creating interface for Scout Mini Omni with AGX_V1 Protocol"
            << std::endl;
      }
    } else if (proto == ProtocolVersion::AGX_V2) {
      std::cout << "Detected protocol: AGX_V2" << std::endl;
      if (!is_omni_wheel_) {
        is_omni_ = false;
        robot_ = std::make_shared<ScoutRobot>(ProtocolVersion::AGX_V2,
                                              is_scout_mini_);
        std::cout << "Creating interface for Scout with AGX_V2 Protocol"
                  << std::endl;
      } else {
        is_omni_ = true;
        omni_robot_ = std::unique_ptr<ScoutMiniOmniRobot>(
            new ScoutMiniOmniRobot(ProtocolVersion::AGX_V2));
        std::cout
            << "Creating interface for Scout Mini Omni with AGX_V2 Protocol"
            << std::endl;
      }
    } else {
      std::cout << "Detected protocol: UNKONWN" << std::endl;
      return false;
    }
  } else {
    return false;
  }

  return true;
}

void ScoutBaseRos::Stop() { keep_running_ = false; }

void ScoutBaseRos::Run() {
  // instantiate a ROS messenger
  if (!is_omni_) {
    std::unique_ptr<ScoutMessenger<ScoutRobot>> messenger =
        std::unique_ptr<ScoutMessenger<ScoutRobot>>(
            new ScoutMessenger<ScoutRobot>(robot_, this));

    messenger->SetOdometryFrame(odom_frame_);
    messenger->SetBaseFrame(base_frame_);
    messenger->SetOdometryTopicName(odom_topic_name_);
    if (simulated_robot_) messenger->SetSimulationMode(sim_control_rate_);

    // connect to robot and setup ROS subscription
    if (port_name_.find("can") != std::string::npos) {
      if (robot_->Connect(port_name_)) {
        robot_->EnableCommandedMode();
        std::cout << "Using CAN bus to talk with the robot" << std::endl;
      } else {
        std::cout << "Failed to connect to the robot CAN bus" << std::endl;
        return;
      }
    } else {
      std::cout << "Please check the specified port name is a CAN port"
                << std::endl;
      return;
    }

    // publish robot state at 50Hz while listening to twist commands
    messenger->SetupSubscription();
    keep_running_ = true;
    rclcpp::Rate rate(50);
    while (keep_running_) {
      messenger->PublishStateToROS();
      rclcpp::spin_some(shared_from_this());
      rate.sleep();
    }
  } else {
    std::unique_ptr<ScoutMessenger<ScoutMiniOmniRobot>> messenger =
        std::unique_ptr<ScoutMessenger<ScoutMiniOmniRobot>>(
            new ScoutMessenger<ScoutMiniOmniRobot>(omni_robot_, this));

    messenger->SetOdometryFrame(odom_frame_);
    messenger->SetBaseFrame(base_frame_);
    messenger->SetOdometryTopicName(odom_topic_name_);
    if (simulated_robot_) messenger->SetSimulationMode(sim_control_rate_);

    // connect to robot and setup ROS subscription
    if (port_name_.find("can") != std::string::npos) {
      if (omni_robot_->Connect(port_name_)) {
        omni_robot_->EnableCommandedMode();
        std::cout << "Using CAN bus to talk with the robot" << std::endl;
      } else {
        std::cout << "Failed to connect to the robot CAN bus" << std::endl;
        return;
      }
    } else {
      std::cout << "Please check the specified port name is a CAN port"
                << std::endl;
      return;
    }

    // publish robot state at 50Hz while listening to twist commands
    messenger->SetupSubscription();
    rclcpp::Rate rate(50);
    keep_running_ = true;
    while (keep_running_) {
      messenger->PublishStateToROS();
      rclcpp::spin_some(shared_from_this());
      rate.sleep();
    }
  }
}
}  // namespace westonrobot
