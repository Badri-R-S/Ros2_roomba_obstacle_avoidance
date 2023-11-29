/************************************************************************************
 * Apache License 2.0
 * Copyright (c) 2021,Badrinarayanan Raghunathan Srikumar
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ************************************************************************************/
/**
 *  @file    obstacle_avoidance.cpp
 *  @author  Badrinarayanan Raghunathan Srikumar
 *  @date    12/05/2022
 *  @version 1.0
 *
 *  @brief Obstacle avoidance using turtlebot3
 *
 *
 *  
 *
 */


#include <algorithm>
#include <cmath>
#include <cstdio>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>


using std::placeholders::_1;

using LASER = sensor_msgs::msg::LaserScan;
using TWIST = geometry_msgs::msg::Twist;

using namespace std::chrono_literals;

/**
 * @brief  States of the robot
*/
typedef enum {
  MOTION = 0,
  STOP,
  ROTATE,
} StateType;

/**
 * @brief ObstacleAvoidance Class
 *
 */
class ObstacleAvoidance : public rclcpp::Node {
 public:
    // Giving STOP as intial state
    ObstacleAvoidance() : Node("walker"), state(STOP) {
    auto p_topic_name = "cmd_vel";
    // Publisher to publish TWIST type of message to the topic
    publisher_ = this->create_publisher<TWIST>(p_topic_name, 10);

    auto default_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
    // subscriber to subscribe to /scan topic
    auto s_topic_name = "/scan";
    // callback function for subscriber
    auto sub_callback = std::bind(&ObstacleAvoidance::subscribe_callback, this, _1);
    subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan> (
        s_topic_name, default_qos, sub_callback);

    // buffer timer for processing
    auto timer = std::bind(&ObstacleAvoidance::callback, this);
    timer_ =
    this-> create_wall_timer(100ms, timer);
  }

 private:
  void subscribe_callback(const LASER& msg) { current_scan = msg; }

// Callback function for subscriber
  void callback() {
    if (current_scan.header.stamp.sec == 0) {
      return;
    }
  // define a TWIST type of publish message
  auto pub = TWIST();
    switch (state) {
      case STOP:
      // If STOP due to obstacle, then turn else move straight
        if (Obstacle() == true) {
          state = ROTATE;
          pub.angular.z = -0.1;
          publisher_->publish(pub);
          RCLCPP_INFO_STREAM(this->get_logger(),
          "STOP state");
        } else {
          state = MOTION;
          pub.linear.x = 0.1;
          publisher_->publish(pub);
          RCLCPP_INFO_STREAM(this->get_logger(),
          "STOP state");
        }
      break;
      
      case MOTION:
        if (Obstacle()== true) {  // check transition
          state = STOP;
          pub.linear.x = 0;
          publisher_->publish(pub);
          RCLCPP_INFO_STREAM(this->get_logger(),
          "MOTION state");
        }
        break;
      
      case ROTATE:
      // If it is in ROTATE State rotate till no obstacle is found
        if (!Obstacle()) {
          state = MOTION;
          pub.linear.x = 0.1;
          publisher_->publish(pub);
          RCLCPP_INFO_STREAM(this->get_logger(),
          "ROTATE state");
        }
        break;
    }
  }

  /**
   * @brief Function to tell if obstacle is found
   *
   *
   */

  bool Obstacle() {
    for (long unsigned int i = 0;
    i < sizeof(current_scan.ranges)/sizeof(current_scan.ranges[0]); i++) {
      if (current_scan.ranges[i] > current_scan.range_min
      && current_scan.ranges[i] < current_scan.range_max) {
                RCLCPP_INFO(this->get_logger(),
                "Distance: %f is valid", current_scan.ranges[i]);
                if (current_scan.ranges[i] < 1.0) {
                RCLCPP_INFO(this->get_logger(),
                "Obstacle detected, rotating");
                return true;
                }
      }

      return false;
    }
    return false;
  }
  // Initialization of publisher, subscriber and variables
  rclcpp::Subscription<LASER>::SharedPtr subscriber_;
  rclcpp::Publisher<TWIST>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  LASER current_scan;
  StateType state;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleAvoidance>());
  rclcpp::shutdown();
  return 0;
}
