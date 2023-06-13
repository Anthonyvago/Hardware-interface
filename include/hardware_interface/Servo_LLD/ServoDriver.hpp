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
#include "Serial_LLD/SerialServoCommunication.hpp"

using namespace std;

class ServoDriver {
public:
  enum Servos : int16_t {
    BASE = 0,
    SHOULDER = 1,
    ELBOW = 2,
    WRIST = 3,
    GRIPPER = 4,
    WRIST_ROTATE = 5,
    ENUM_END = 6
  };

  ServoDriver(const string &pathToDevice);

  ~ServoDriver() = default;

  void setServoDegree(Servos servo, int16_t degree, int16_t time = 0);

  int16_t getServoDegree(Servos servo);

  void disengageEmergencyStop();

  void engageEmergencyStop();

  int getServoCount();

private:
  int servoCount_;

  bool emergencyStopEngaged_;

  SerialServoCommunication serialServos_;

  map<int16_t, int16_t> servoMap_;

  map<ServoDriver::Servos, pair<pair<int16_t, int16_t>, pair<int16_t, int16_t>>> ranges_;

};

#endif // SERVODRIVER_HPP