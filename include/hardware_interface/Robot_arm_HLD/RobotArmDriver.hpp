/**
 * @file RobotArmDriver.hpp
 * @author Anthony Vágó (A.Vago@student.han.nl)
 * @brief
 * @version 0.1
 * @date 2023-06-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef ROBOTARMDRIVER_HPP
#define ROBOTARMDRIVER_HPP

#include <Servo_LLD/ServoDriver.hpp>
#include <hardware_interface/msg/set_servos.hpp>
#include <hardware_interface/msg/set_robot_arm_state.hpp>
// #include <hardware_interface/srv/get_servos.hpp>
// #include <hardware_interface/srv/get_robot_arm_state.hpp>

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <queue>
#include <chrono>
#include <algorithm>
#include <math.h>

using namespace std;

// When no time is given, the default time is used:
#define DEFAULT_EXECUTION_TIME 2000 // In milliseconds

// There are two kinds of events, one for setting the state and one for setting
// the servos. So for both I made a struct with the data that is needed:
typedef enum { STATE, SERVO } EventType;

// General event structure:
struct Event {
  EventType eventType;
  Event(EventType event) : eventType(event) {}
  virtual EventType getEventType() { return eventType; };
  virtual ~Event() = default;
};

// One event structure for a servo event:
struct EventServo : public Event {
  EventServo(EventType eventType, vector<uint16_t> servos, vector<int16_t> degrees) : Event(eventType), servos(servos), degrees(degrees), times(servos.size(), DEFAULT_EXECUTION_TIME) {};
  EventServo(EventType eventType, vector<uint16_t> servos, vector<int16_t> degrees, vector<uint16_t> times) : Event(eventType), servos(servos), degrees(degrees), times(times) {};
  vector<uint16_t> servos;
  vector<int16_t> degrees;
  vector<uint16_t> times;
  virtual ~EventServo() = default;
};

// Robot arm states:
enum RobotArmState : uint16_t {
  READY = 0,
  PARK = 1,
  STRAIGHT_UP = 2,
  EMERGENCY_STOP = 3
} ;

// One event structure for a state event:
struct EventState : public Event {
  EventState(EventType eventType, RobotArmState state) : Event(eventType), desiredState(state) {}
  RobotArmState desiredState;
  EventType getEventType() { return Event::eventType; };
  virtual ~EventState() = default;
};



class RobotArmDriver : public rclcpp::Node {
public:
  RobotArmDriver();

  ~RobotArmDriver();

  /**
   * @brief Adds servo event to the queue.
   * 
   * @param msg Received message from the topic.
   */
  void setServos(const hardware_interface::msg::SetServos::SharedPtr msg);

  /**
   * @brief Adds state event to the queue.
   * 
   * @param msg Received message from the topic.
   */
  void setRobotArmState(const hardware_interface::msg::SetRobotArmState::SharedPtr msg);

  /**
   * @brief Timer callback function.
   * 
   */
  void runQueue();

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<hardware_interface::msg::SetServos>::SharedPtr servoSub_;
  rclcpp::Subscription<hardware_interface::msg::SetRobotArmState>::SharedPtr stateSub_;
  ServoDriver servoDriver_;
  queue<shared_ptr<Event>> eventsQueue_;
  RobotArmState curState_;
  vector<uint64_t> servoActivationTimes_;
};

#endif