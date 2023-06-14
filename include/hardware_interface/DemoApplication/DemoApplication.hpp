/**
 * @file DemoApplication.hpp
 * @author Anthony Vágó (A.Vago@student.han.nl)
 * @brief 
 * @version 0.1
 * @date 2023-06-12
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef DEMOAPPLICATION_HPP
#define DEMOAPPLICATION_HPP

#include <Robot_arm_HLD/RobotArmDriver.hpp>
#include <hardware_interface/msg/set_servos.hpp>
#include <hardware_interface/msg/set_robot_arm_state.hpp>
// #include <hardware_interface/srv/get_servos.hpp>
// #include <hardware_interface/srv/get_robot_arm_state.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std;

class DemoApplication : public rclcpp::Node {
public:
  /**
   * @brief Constructor function.
   */
  DemoApplication();

  /**
   * @brief This timer callback is called every 100ms and interprets commands.
   * @return (void)
   */
  void timerCallback();

  /**
   * @brief 
   * @param servos list of servo ids to be actuated
   * @param degree list of positions corresponding to ids in servos
   * @param times list of durations indicating how long a servo should take to
   * move to a new position
   * @return (void)
   */
  void setServoDegrees(const vector<unsigned short> &servos,
                       const vector<short> &degree,
                       const vector<unsigned short> &times); // set servo degree

  /**
   * @brief sends command to robot to be put into a predefined position
   * corresponding to a state
   * @param state state to put the robot into
   * @return (void)
   */
  void setRobotArmState(RobotArmState state); // set state

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<hardware_interface::msg::SetServos>::SharedPtr servoPub_;
  rclcpp::Publisher<hardware_interface::msg::SetRobotArmState>::SharedPtr statePub_;
};

#endif
