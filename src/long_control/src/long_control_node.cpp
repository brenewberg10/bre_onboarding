/* Copyright 2021 Will Bryan

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

#include <long_control.hpp>

namespace control {
LongControl::LongControl() : Node("LongControlNode") {
  // Publishers

    
  this->pubThrottleCmd_ =
      this->create_publisher<std_msgs::msg::Float32>("/joystick/accelerator_cmd", 1);
  this->pubBrakeCmd_ = this->create_publisher<std_msgs::msg::Float32>("/joystick/brake_cmd", 1);
  this->pubGearCmd_ = this->create_publisher<std_msgs::msg::Int8>("/joystick/gear_cmd", 1);

  this->pubHeartbeat_ = this->create_publisher<std_msgs::msg::Empty>("/control/long_control_heartbeat",1);
  
  // setup QOS to be best effort
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
  qos.best_effort();

  // Subscribers
  // this->subVelocity_ = this->create_subscription<raptor_dbw_msgs::msg::WheelSpeedReport>(
  //     "/raptor_dbw_interface/wheel_speed_report", 1,
  //     std::bind(&LongControl::receiveVelocity, this, std::placeholders::_1));
  this->subVelocity_ = this->create_subscription<autonoma_msgs::msg::VehicleData>(
      "/vehicle_data", 1,
      std::bind(&LongControl::receiveVelocity, this, std::placeholders::_1));
  // this->subPtReport_ = this->create_subscription<deep_orange_msgs::msg::PtReport>(
  //     "/raptor_dbw_interface/pt_report", 1,
  //     std::bind(&LongControl::receivePtReport, this, std::placeholders::_1));
  this->subPtReport_ = this->create_subscription<autonoma_msgs::msg::PowertrainData>(
      "/powertrain_data", 1,
      std::bind(&LongControl::receivePtReport, this, std::placeholders::_1));
  this->subDesiredVelocity_ = this->create_subscription<std_msgs::msg::Float32>(
      "/planning/desired_velocity", qos,
      std::bind(&LongControl::receiveDesiredVelocity, this, std::placeholders::_1));
  
  
  

  // Declare Parameters
  // this->declare_parameter("desired_velocity", 12.5); // DEPRECATED now that desired velocity is
  // topic based
  this->declare_parameter("time_step", 0.01);
  this->declare_parameter("mute", false);
  this->declare_parameter("max_desired_velocity", 25.0);  // Temp max velocity desired

  //FIXME: I believe this serves no purpose anymore
  this->declare_parameter("override_absent_joystick",
                          true);  // drive even when there is no joystick data being published

  this->declare_parameter("throttle.proportional_gain", 4.0);
  this->declare_parameter("throttle.integral_gain", 0.0);
  this->declare_parameter("throttle.derivative_gain", 0.0);
  this->declare_parameter("throttle.max_integrator_error", 10.0);
  this->declare_parameter("throttle.cmd_max", 25.0);
  this->declare_parameter("throttle.cmd_min", 0.0);
  this->declare_parameter("throttle.reset_integral_below_this_cmd", 15.0);

  this->declare_parameter("brake.proportional_gain", 4.0);
  this->declare_parameter("brake.integral_gain", 0.0);
  this->declare_parameter("brake.derivative_gain", 0.0);
  this->declare_parameter("brake.max_integrator_error", 10.0);
  this->declare_parameter("brake.park_brake", 310.0);
  this->declare_parameter("brake.cmd_max", 25.0);
  this->declare_parameter("brake.cmd_min", 0.0);
  this->declare_parameter("brake.reset_integral_below_this_cmd", 15.0);
  this->declare_parameter("brake.vel_error_deadband", 0.5);

  this->declare_parameter("gear.shift_up", 10.0);
  this->declare_parameter("gear.shift_down", 9.0);
  this->declare_parameter("gear.shift_time_ms", 300);

  this->declare_parameter("max_one_step_long_err_rate", 5000.0);
  this->declare_parameter("max_avg_long_err_rate", 2500.0);
  this->declare_parameter("long_err_history_length", 4);

  

  // Create Callback Timers
  this->ts_ = this->get_parameter("time_step").as_double();
  int timer_ms_ = static_cast<int>(ts_ * 1000);
  this->control_timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_ms_),
                                                 std::bind(&LongControl::controlCallback, this));

  int timer_param_ms_ = static_cast<int>(ts_ * 10000);
  this->param_timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_param_ms_),
                                               std::bind(&LongControl::paramUpdateCallback, this));

  int timer_gear_ms_ = static_cast<int>(ts_ * 10000);
  this->gear_timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_gear_ms_),
                                              std::bind(&LongControl::shiftCallback, this));

  // Initialize Commands
  this->throttle_cmd.data = 0.0;
  this->brake_cmd.data = 0.0;
  this->gear_cmd.data = 1;

  // Create throttle PID object
  this->p_ = this->get_parameter("throttle.proportional_gain").as_double();
  this->i_ = this->get_parameter("throttle.integral_gain").as_double();
  this->d_ = this->get_parameter("throttle.derivative_gain").as_double();
  this->iMax_ = this->get_parameter("throttle.max_integrator_error").as_double();
  this->throttleCmdMax_ = this->get_parameter("throttle.cmd_max").as_double();
  this->throttleCmdMin_ = this->get_parameter("throttle.cmd_min").as_double();
  this->iThrottleReset_ = this->get_parameter("throttle.reset_integral_below_this_cmd").as_double();

  this->vel_pid_ = PID(p_, i_, d_, ts_, iMax_, throttleCmdMax_, throttleCmdMin_);

  // Create brake PID object
  this->bp_ = this->get_parameter("brake.proportional_gain").as_double();
  this->bi_ = this->get_parameter("brake.integral_gain").as_double();
  this->bd_ = this->get_parameter("brake.derivative_gain").as_double();
  this->biMax_ = this->get_parameter("brake.max_integrator_error").as_double();
  this->brakeCmdMax_ = this->get_parameter("brake.cmd_max").as_double();
  this->brakeCmdMin_ = this->get_parameter("brake.cmd_min").as_double();
  this->iBrakeReset_ = this->get_parameter("brake.reset_integral_below_this_cmd").as_double();

  this->brake_pid_ = PID(bp_, bi_, bd_, ts_, biMax_, brakeCmdMax_, brakeCmdMin_);
}

void LongControl::controlCallback() {
  double error = calculateVelocityError();
  calculateThrottleCmd(error);
  calculateBrakeCmd(error);

  publishThrottleBrake();

  this->pubHeartbeat_->publish(this->heartbeatMsg);
  publishDebugSignals();
}

void LongControl::paramUpdateCallback() {
  this->vel_pid_.SetPGain(this->get_parameter("throttle.proportional_gain").as_double());
  this->vel_pid_.SetIGain(this->get_parameter("throttle.integral_gain").as_double());
  this->vel_pid_.SetDGain(this->get_parameter("throttle.derivative_gain").as_double());
  this->vel_pid_.SetIMax(this->get_parameter("throttle.max_integrator_error").as_double());
  this->vel_pid_.SetCmdBounds(this->get_parameter("throttle.cmd_min").as_double(),
                              this->get_parameter("throttle.cmd_max").as_double());

  this->brake_pid_.SetPGain(this->get_parameter("brake.proportional_gain").as_double());
  this->brake_pid_.SetIGain(this->get_parameter("brake.integral_gain").as_double());
  this->brake_pid_.SetDGain(this->get_parameter("brake.derivative_gain").as_double());
  this->brake_pid_.SetIMax(this->get_parameter("brake.max_integrator_error").as_double());
  this->brake_pid_.SetCmdBounds(this->get_parameter("brake.cmd_min").as_double(),
                                this->get_parameter("brake.cmd_max").as_double());
}

double LongControl::calculateVelocityError() {
  rclcpp::Duration time_diff = rclcpp::Clock().now() - this->vel_recv_time_;
  double dt = static_cast<double>(time_diff.seconds()) +
              static_cast<double>(time_diff.nanoseconds()) * 1e-9;

  if (dt > 100 * this->ts_) {
    this->vel_pid_.ResetErrorIntegral();
  }

  double des_vel = this->desired_velocity;  // this->get_parameter("desired_velocity").as_double();
  double max_des_vel = this->get_parameter("max_desired_velocity").as_double();
  double v_des = std::min(des_vel, max_des_vel);
  double vel_error = v_des - this->speed_;

  int longerrhistSize = this->get_parameter("long_err_history_length").as_int();

  if (longerrhistSize != this->long_err_history.size()) {
    if (longerrhistSize < this->long_err_history.size())
      this->long_err_history_counter = longerrhistSize;
    this->long_err_history.resize(longerrhistSize,0.0);
    this->long_err_time_history.resize(longerrhistSize,rclcpp::Clock().now());
  }

  if (this->long_err_history_counter > this->long_err_history.size()-1) {
    for (int index = 0; index < long_err_history.size()-1; ++index){
      this->long_err_history[index] = this->long_err_history[index+1];
      this->long_err_time_history[index] = this->long_err_time_history[index+1];
    }
    this->long_err_history[longerrhistSize-1] = vel_error;
    this->long_err_time_history[longerrhistSize-1] = rclcpp::Clock().now();
  }
  else{
    this->long_err_history[this->long_err_history_counter] = vel_error;
    this->long_err_time_history[this->long_err_history_counter] = rclcpp::Clock().now();
    this->long_err_history_counter++;
  }  

  // RCLCPP_INFO(this->get_logger(),"Velocity_error: %f", vel_error);
  return vel_error;
}



void LongControl::calculateThrottleCmd(double vel_err) {
  if (vel_err > 0.0) {
    this->vel_pid_.Update(vel_err);
    this->throttle_cmd.data = this->vel_pid_.CurrentControl();
  } else {
    this->throttle_cmd.data = 0.0;
  }
}

void LongControl::receiveDesiredVelocity(const std_msgs::msg::Float32::SharedPtr msg) {
  this->desired_velocity = msg->data;
}

void LongControl::calculateBrakeCmd(double vel_err) {
  double db = this->get_parameter("brake.vel_error_deadband").as_double();
  if (vel_err < -db) {
    this->brake_pid_.Update(-vel_err);
    this->brake_cmd.data = this->brake_pid_.CurrentControl();
  } else {
    this->brake_cmd.data = 0.0;
    this->brake_pid_.ResetErrorIntegral();
  }
}

void LongControl::setCmdsToZeros() {
  this->throttle_cmd.data = 0.0;
  this->brake_cmd.data = 0.0;
}

void LongControl::publishThrottleBrake() {
  if (this->speed_ < 1.0 && this->desired_velocity == 0.0) {
    this->brake_cmd.data = this->get_parameter("brake.park_brake").as_double();
    this->throttle_cmd.data = 0.;
  } else if (this->brake_cmd.data > 0.0) {
    this->throttle_cmd.data = 0.0;
  }

  this->control_cmd_msg.accelerator_cmd = this->throttle_cmd.data;
  this->control_cmd_msg.frt_brake_cmd = this->brake_cmd.data;
  this->control_cmd_msg.rear_brake_cmd = this->brake_cmd.data;
  this->control_cmd_msg.gear_cmd = this->gear_cmd.data;

  if (!(this->get_parameter("mute").as_bool())) {
    pubThrottleCmd_->publish(this->throttle_cmd);
    pubBrakeCmd_->publish(this->brake_cmd);
  }
}

void LongControl::publishDebugSignals() {

  if (this->long_err_history_counter > this->long_err_history.size()-1) { 
    int longerrhistSize = this->long_err_history.size();

    double max_one_step_long_err_rate = this->get_parameter("max_one_step_long_err_rate").as_double();
    double max_avg_long_err_rate = this->get_parameter("max_avg_long_err_rate").as_double();

    rclcpp::Duration dt = this->long_err_time_history[longerrhistSize-1] - this->long_err_time_history[longerrhistSize-2];
    double one_step_long_err_rate = (this->long_err_history[longerrhistSize-1] - this->long_err_history[longerrhistSize-2])/(dt.seconds() + dt.nanoseconds()*1e-9 + __DBL_EPSILON__);
    dt = this->long_err_time_history[longerrhistSize-1] - this->long_err_time_history[0];
    double avg_long_err_rate = (*std::max_element(this->long_err_history.begin(),this->long_err_history.end()) - *std::min_element(this->long_err_history.begin(),this->long_err_history.end()))/(dt.seconds() + dt.nanoseconds()*1e-9 + __DBL_EPSILON__); 
  }
  
}

void LongControl::shiftCallback() {
  double upshift_rpm = this->get_parameter("gear.shift_up").as_double();
  double downshift_rpm = this->get_parameter("gear.shift_down").as_double();
  unsigned int shift_time_limit = this->get_parameter("gear.shift_time_ms").as_int();

  // Sets command to current gear if engine is not on or shift attempts denied over the limit
  if (!this->engine_running_ || this->shifting_counter_ * 100 >= shift_time_limit) {
    this->gear_cmd.data = this->current_gear_;
    this->shifting_counter_ = 0;
    if (!(this->get_parameter("mute").as_bool())) pubGearCmd_->publish(this->gear_cmd);
    return;
  }

  if (this->engine_speed_ > upshift_rpm && this->throttle_cmd.data > 0.0) {
    this->gear_cmd.data = std::min(this->current_gear_ + 1, 6);
    this->shifting_counter_++;
  } else if (this->engine_speed_ < downshift_rpm) {
    this->gear_cmd.data = std::max(this->current_gear_ - 1, 1);
    this->shifting_counter_++;
  } else {
    this->gear_cmd.data = this->current_gear_;
    this->shifting_counter_ = 0;
  }
  if (!(this->get_parameter("mute").as_bool())) pubGearCmd_->publish(this->gear_cmd);
}

// void LongControl::receiveVelocity(const raptor_dbw_msgs::msg::WheelSpeedReport::SharedPtr msg) {
//   const double kphToMps = 1.0 / 3.6;
//   double rear_left = msg->rear_left;
//   double rear_right = msg->rear_right;
//   this->speed_ = (rear_left + rear_right) * 0.5 * kphToMps;  // average wheel speeds (kph) and convert to m/s
//   this->vel_recv_time_ = rclcpp::Clock().now();
// }

void LongControl::receiveVelocity(const autonoma_msgs::msg::VehicleData::SharedPtr msg) {
  const double kphToMps = 1.0 / 3.6;
  double rear_left = msg->ws_rear_left;
  double rear_right = msg->ws_rear_right;
  this->speed_ = (rear_left + rear_right) * 0.5 * kphToMps;  // average wheel speeds (kph) and convert to m/s
  this->vel_recv_time_ = rclcpp::Clock().now();
}

// void LongControl::receivePtReport(const deep_orange_msgs::msg::PtReport::SharedPtr msg) {
//   this->current_gear_ = msg->current_gear;
//   this->engine_speed_ = msg->engine_rpm;
//   this->engine_running_ = msg->engine_on_status;
//   // this->speed_ = msg->vehicle_speed_kmph / 3.6;
// }

void LongControl::receivePtReport(const autonoma_msgs::msg::PowertrainData::SharedPtr msg) {
  this->current_gear_ = msg->current_gear;
  this->engine_speed_ = msg->engine_rpm;
  this->engine_running_ = msg->engine_on_status;
}  
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<control::LongControl>());
  rclcpp::shutdown();
  return 0;
}
