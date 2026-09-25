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

#include <kin_control.hpp>

namespace control {

void KinControl::controlCallback() {
  rclcpp::Time control_time = rclcpp::Clock().now();
  rclcpp::Duration time_diff = control_time - this->recv_time_;
  double age = static_cast<double>(time_diff.seconds()) +
               static_cast<double>(time_diff.nanoseconds()) * 1e-9;

  if (age < 10 * dt) {
    calculateFFW();
    calculateFB(dt);
    calculateSteeringCmd();
  } else {
    RCLCPP_DEBUG(this->get_logger(), "%s\n", "Have not received new path in > 0.5s !");
    setCmdsToZeros();
  }

  publishSteering();
  publishDebugSignals();
}

void KinControl::calculateFFW() {
  // Desired yaw rate from feedforward
  this->feedforward_ =
      0.0;  // can add feedforward with a curvature: this->speed_ * this->curvature_;
}

void KinControl::calculateFB(double dt) {
  static double derivative = 0.;
  // Desired yaw rate from feedback
  double Kp = this->get_parameter("proportional_gain").as_double();
  double Kd = this->get_parameter("derivative_gain").as_double();

  double del = look_ahead_error_history[lae_history_counter];
  look_ahead_error_history[lae_history_counter] = this->lookahead_error.data;
  lae_history_counter = (lae_history_counter + 1) % 4;
  double prev_lae_history_avg = lae_history_avg;
  lae_history_avg = lae_history_avg - 0.25 * del + 0.25 * this->lookahead_error.data;

  // perform an exponential filter on derivative term
  derivative = (lae_history_avg - prev_lae_history_avg) / dt;

  this->feedback_ = Kp * this->lookahead_error.data + Kd * derivative;
}

void KinControl::calculateSteeringCmd() {
  // Convert desired yaw rate to steering angle using kinematic model
  double L = this->get_parameter("vehicle.wheelbase").as_double();
  this->steering_cmd.data =
      L * (this->feedback_ + this->feedforward_) / std::fmax(this->speed_, 1.);
  this->steering_cmd.data = this->steering_cmd.data * 180.0 / M_PI;
}

void KinControl::setCmdsToZeros() {
  this->feedforward_ = 0.0;
  this->feedback_ = 0.0;
  this->lookahead_error.data = 0.0;
  this->curvature_ = 0.0;
  this->speed_ = 0.0;

  // reset moving average filter for derivative
  for (int i = 0; i < 4; i++) this->look_ahead_error_history[i] = 0.0;
  lae_history_avg = 0.;

  this->steering_cmd.data = 0.0;
}

void KinControl::publishSteering() {
  double ms = this->get_parameter("max_steer_angle").as_double();

  if (this->steering_cmd.data > ms) {
    RCLCPP_DEBUG(this->get_logger(), "%s\n", "Saturation Limit Max");
    this->steering_cmd.data = ms;
  }
  if (this->steering_cmd.data < -ms) {
    RCLCPP_DEBUG(this->get_logger(), "%s\n", "Saturation Limit Min");
    this->steering_cmd.data = -ms;
  }

  this->control_cmd_msg.steering_cmd = this->steering_cmd.data;
  if (!this->get_parameter("mute").as_bool()) {
    pubSteeringCmd_->publish(this->steering_cmd);
    // pubControlCmd_->publish(this->control_cmd_msg);
  }
}

void KinControl::publishDebugSignals() {
  // check for too large lookahead error and publish to safety_node
  float lookahead_error_fabs = fabs(this->lookahead_error.data);
  // check for too large lateral error
  float lateral_error_fabs = fabs(this->lat_error.data);
  filtered_lateral_error_ = filtered_lateral_error_ * 0.5 + lateral_error_fabs * 0.5;
  std_msgs::msg::Float64 filt_lat_error;
  filt_lat_error.data = filtered_lateral_error_;
  pubFiltLatError_->publish(filt_lat_error);

  pubLookaheadError_->publish(this->lookahead_error);
  pubLatError_->publish(this->lat_error);
}

float _dist(float x, float y, float z) {
        return std::sqrt(std::pow(x,2) + std::pow(y,2) + std::pow(z,2));
}

void KinControl::receivePath(const nav_msgs::msg::Path::SharedPtr msg) {

  this->path_msg = msg;
  // Determines lookahead distance based on speed and bounds

  double min_la = this->get_parameter("min_lookahead").as_double();
  double max_la = this->get_parameter("max_lookahead").as_double();
  double la_ratio = this->get_parameter("lookahead_speed_ratio").as_double();
  double lookahead_distance = std::max(min_la, std::min(max_la, this->speed_ * la_ratio));

  // Unpacks the message and finds the index correlated to the lookahead distance
  std::vector<geometry_msgs::msg::PoseStamped> path = msg->poses;
  // Sets the lookahead and lateral error
  if (!path.size()) {
    // an empty path message is NOT a valid path. Do NOT update clock in this case.
    this->lookahead_error.data = 0.0;
    return;
  }
  int idx;
  double fraction;
  std::tie(idx, fraction) = findLookaheadIndex(path, lookahead_distance);
  if (idx >= static_cast<int>(path.size()) - 1) {
    this->lookahead_error.data = path[idx].pose.position.y;
  } else {
    double idx1_y = path[idx + 1].pose.position.y;
    double idx_y = path[idx].pose.position.y;
    this->lookahead_error.data = idx_y * fraction + idx1_y * (1. - fraction);
  }
  if (path.size() == 1) {
    // this->lat_error.data = path[0].pose.position.y;
    this->lat_error.data = computeLateralError(path[0]);
  } else {
    double diff_x = path[1].pose.position.x - path[0].pose.position.x;
    double diff_y = path[1].pose.position.y - path[0].pose.position.y;
    double norm_factor = 1. / std::sqrt(diff_x * diff_x + diff_y * diff_y);
    double norm_x = diff_x * norm_factor;
    double norm_y = diff_y * norm_factor;
    // this->lat_error.data = path[0].pose.position.y;//-path[0].pose.position.x * norm_y + path[0].pose.position.y * norm_x;
    this->lat_error.data = computeLateralError(path[0]);
  }

  this->recv_time_ = rclcpp::Clock().now();
}

double KinControl::computeLateralError(const geometry_msgs::msg::PoseStamped& pathPose1) {
    
    double x_p1 = pathPose1.pose.position.x;
    double y_p1 = pathPose1.pose.position.y;

    tf2::Quaternion q;
    tf2::fromMsg(pathPose1.pose.orientation, q);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    yaw = wrap_angle(yaw);
    double path_dx = cos(yaw);
    double path_dy = sin(yaw);

    double norm_x = -path_dy;
    double norm_y = path_dx;

    double norm_factor = 1.0 / std::sqrt(norm_x * norm_x + norm_y * norm_y);
    norm_x *= norm_factor;
    norm_y *= norm_factor;

    double distance = norm_x * (-x_p1) + norm_y * (-y_p1);

    return - distance; // add a negative sign here to make the lateral error positive when the vehicle is to the right of the path, align with previous implementation
}


std::tuple<int, double> KinControl::findLookaheadIndex(
    std::vector<geometry_msgs::msg::PoseStamped> refPath, double desLookaheadValue) {
  // calculate frenet distance iteratively.
  double cumulativeDist;
  if (refPath.size() == 1) {
    cumulativeDist = 0.;
  } else {
    double diff_x = refPath[1].pose.position.x - refPath[0].pose.position.x;
    double diff_y = refPath[1].pose.position.y - refPath[0].pose.position.y;
    double norm_factor = 1. / std::sqrt(diff_x * diff_x + diff_y * diff_y);
    double norm_x = diff_x * norm_factor;
    double norm_y = diff_y * norm_factor;
    cumulativeDist = refPath[0].pose.position.x * norm_x + refPath[0].pose.position.y * norm_y;
  }
  double lastCumulativeDist = 0.;
  size_t i;
  for (i = 0; i < (refPath.size() - 1) && cumulativeDist < desLookaheadValue; i++) {
    double diffX = refPath[i + 1].pose.position.x - refPath[i].pose.position.x;
    double diffY = refPath[i + 1].pose.position.y - refPath[i].pose.position.y;
    lastCumulativeDist = cumulativeDist;
    cumulativeDist += std::sqrt(diffX * diffX + diffY * diffY);
  }
  return {i, (cumulativeDist - desLookaheadValue) /
                 (cumulativeDist - lastCumulativeDist + __DBL_EPSILON__)};
}


// void KinControl::receiveVelocity(const raptor_dbw_msgs::msg::WheelSpeedReport::SharedPtr msg) {
//   const double kphToMps = 1.0 / 3.6;
//   double rear_left = msg->rear_left;
//   double rear_right = msg->rear_right;
//   this->speed_ =
//       (rear_left + rear_right) * 0.5 * kphToMps;  // average wheel speeds (kph) and convert to m/s
// }

void KinControl::receiveVelocity(const autonoma_msgs::msg::VehicleData::SharedPtr msg) {
  const double kphToMps = 1.0 / 3.6;
  double rear_left = msg->ws_rear_left;
  double rear_right = msg->ws_rear_right;
  this->speed_ =
      (rear_left + rear_right) * 0.5 * kphToMps;  // average wheel speeds (kph) and convert to m/s
}

}  // end namespace control

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<control::KinControl>());
  rclcpp::shutdown();
  return 0;
}
