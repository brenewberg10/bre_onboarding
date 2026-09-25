/* Copyright 2021 Will Bryan
   Copyright 2022-2024 Black and Gold Autonomous Racing
   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/

#ifndef KIN_CONTROL_HPP
#define KIN_CONTROL_HPP

#include <math.h>
#include <unistd.h>

#include <chrono>
#include <cstdlib>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <algorithm>

#include "deep_orange_msgs/msg/joystick_command.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "raptor_dbw_msgs/msg/wheel_speed_report.hpp"
#include "autonoma_msgs/msg/vehicle_data.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif




namespace control {

class KinControl : public rclcpp::Node {
 public:
  KinControl() : Node("KinControlNode") {
    // setup QOS to be best effort
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
    qos.best_effort();

    // Parameters
    this->declare_parameter("min_lookahead", 4.0);
    this->declare_parameter("max_lookahead", 50.0);
    this->declare_parameter("lookahead_speed_ratio", 0.75);
    this->declare_parameter("proportional_gain", 0.2);
    this->declare_parameter("derivative_gain", 0.02);
    this->declare_parameter("vehicle.wheelbase", 2.97);
    this->declare_parameter("max_steer_angle", 15.0); 
    this->declare_parameter("max_lat_error", 4.0);
    this->declare_parameter("max_lookahead_error", 8.0);
    this->declare_parameter("auto_enabled", true);
    this->declare_parameter("mute", false);
    this->declare_parameter("curv_to_velocity_constant", 10000.0);
    this->declare_parameter("max_possible_deceleration", 5.0); // m/s/s should be same as graceful_stop_acceleration in speed_reference_generator

    // Publishers
    // pubControlCmd_ = this->create_publisher<deep_orange_msgs::msg::JoystickCommand>("/control/pid/kin_command", qos.best_effort());
    pubSteeringCmd_ = this->create_publisher<std_msgs::msg::Float32>("/joystick/steering_cmd", 1);
    pubLookaheadError_ = this->create_publisher<std_msgs::msg::Float64>("/control/pid/lookahead_error", 10);
    pubLatError_ = this->create_publisher<std_msgs::msg::Float64>("/control/pid/lateral_error", 10);
    pubFiltLatError_ = this->create_publisher<std_msgs::msg::Float64>("/control/pid/filtered_lateral_error", 10);

    // Subscribers
    // TODO: use parameters to specify path topics
    subPath_ = this->create_subscription<nav_msgs::msg::Path>(
        "/planning/front_path/offset_path", 1,
        std::bind(&KinControl::receivePath, this, std::placeholders::_1));

    // subVelocity_ = this->create_subscription<raptor_dbw_msgs::msg::WheelSpeedReport>(
    //     "/raptor_dbw_interface/wheel_speed_report", qos,
    //     std::bind(&KinControl::receiveVelocity, this, std::placeholders::_1));

    subVelocity_ = this->create_subscription<autonoma_msgs::msg::VehicleData>(
        "/vehicle_data", 1,
        std::bind(&KinControl::receiveVelocity, this, std::placeholders::_1));

    // Create Timer
    control_timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(dt * 1000)),
                                             std::bind(&KinControl::controlCallback, this));

    lookahead_error.data = 0.0;
    lat_error.data = 0.0;
    steering_cmd.data = 0.0;

    this->max_lat_error = this->get_parameter("max_lat_error").as_double();
    this->max_lookahead_error = this->get_parameter("max_lookahead_error").as_double();

  };
  
  deep_orange_msgs::msg::JoystickCommand control_cmd_msg;
  std_msgs::msg::Float32 steering_cmd;
  std_msgs::msg::Float64 lookahead_error;
  std_msgs::msg::Float64 lat_error;


 private:
  void controlCallback();
  void calculateFFW();
  void calculateFB(double dt);
  void calculateSteeringCmd();
  void setCmdsToZeros();
  void publishSteering();
  void publishDebugSignals();
  void receivePath(const nav_msgs::msg::Path::SharedPtr msg);
  std::tuple<int, double> findLookaheadIndex(std::vector<geometry_msgs::msg::PoseStamped> refPath,
                                             double desLookaheadValue);
  // void receiveVelocity(const raptor_dbw_msgs::msg::WheelSpeedReport::SharedPtr msg);
  void receiveVelocity(const autonoma_msgs::msg::VehicleData::SharedPtr msg);

  double computeLateralError(const geometry_msgs::msg::PoseStamped& pathPose1);
  double wrap_angle(double angle) noexcept{
        angle = std::fmod(angle + M_PI_2, M_PI);
        if (angle < 0)
            angle += M_PI;
        return angle - M_PI_2;
  }
  rclcpp::TimerBase::SharedPtr control_timer_;
//   rclcpp::Publisher<deep_orange_msgs::msg::JoystickCommand>::SharedPtr pubControlCmd_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubSteeringCmd_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubLookaheadError_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubLatError_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubFiltLatError_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subPath_;
  // rclcpp::Subscription<raptor_dbw_msgs::msg::WheelSpeedReport>::SharedPtr subVelocity_;
  rclcpp::Subscription<autonoma_msgs::msg::VehicleData>::SharedPtr subVelocity_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subPathCurvature;
  rclcpp::Time recv_time_;

  nav_msgs::msg::Path::SharedPtr path_msg;

  double curvature_ = 0.0;
  double curvature_velocity = 0.0;  // The max velocity we may go based on how sharp of turn we are approaching
  double speed_ = 0.0;
  double feedforward_ = 0.0;
  double feedback_ = 0.0;
  double max_lat_error = 0.0;
  double max_lookahead_error = 0.0;
  double dt = 0.01;
  double filtered_lateral_error_ = 0.;


  // apply moving average filter to look ahead history
  double look_ahead_error_history[4] = {0.0};
  int lae_history_counter = 0;
  double lae_history_avg = 0.;


};  // end of class

}  // namespace control

#endif
