#ifndef DECCEL_STOP_HPP
#define DECCEL_STOP_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "raptor_dbw_msgs/msg/wheel_speed_report.hpp"
#include "deep_orange_msgs/msg/pt_report.hpp"
#include "deep_orange_msgs/msg/joystick_command.hpp"

#include "PID.hpp"

namespace control 

class DeccelStop : public rclcpp::Node
{
  public:
    DeccelStop();


  private:
    void paramUpdateCallback();
    void receiveSpeed(const raptor_dbw_msgs::msg::WheelSpeedReport::SharedPtr msg);
    rclcpp::TimerBase::SharedPtr param_timer_;
    
    dobule speed_ 
    

    
