/**
 * @file ServoDriver.hpp
 * @author Anthony Vágó (A.Vago@student.han.nl)
 * @brief
 * @version 0.1
 * @date 2023-06-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef SERVODRIVER_HPP
#define SERVODRIVER_HPP

#include <iostream>
#include <map>
#include "Servo_LLD/SerialServoCommunication.hpp"

using namespace std;

enum Servos : int16_t {
  BASE = 0,
  SHOULDER = 1,
  ELBOW = 2,
  WRIST = 3,
  GRIPPER = 4,
  WRIST_ROTATE = 5,
  ENUM_END = 6
};

class ServoDriver {
public:
  ServoDriver(const string &pathToDevice);
  
  ServoDriver();

  ~ServoDriver() = default;

  void setServoDegrees(Servos servo, int16_t degree, int16_t time = 0);

  int16_t getServoDegrees(Servos servo);

  void disengageEmergencyStop();

  void engageEmergencyStop();

  int getServoCount();

  int16_t degreeToPWM(Servos servo, int16_t degree);

  int16_t PWMtoDegree(Servos servo, int16_t pwm);

private:
  int servoCount_;

  bool emergencyStopEngaged_;

  SerialServoCommunication serialServos_;

  map<int16_t, int16_t> servoMap_;

  map<Servos, pair<pair<int16_t, int16_t>, pair<int16_t, int16_t>>> ranges_;
};

#endif // SERVODRIVER_HPP
