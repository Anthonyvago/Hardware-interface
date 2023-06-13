/**
 * @file ServoDriver.cpp
 * @author Anthony Vágó (A.Vago@student.han.nl)
 * @brief
 * @version 0.1
 * @date 2023-06-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "Servo_LLD/ServoDriver.hpp"

ServoDriver::ServoDriver(const string &pathToDevice)
    : serialServos_(pathToDevice), servoCount_(6),
      emergencyStopEngaged_(false) {
  ranges_.emplace(ServoDriver::Servos::BASE,
                  make_pair(make_pair(600, 2385), make_pair(-90, 90)));
  ranges_.emplace(ServoDriver::Servos::SHOULDER,
                  make_pair(make_pair(1750, 750), make_pair(-30, 90)));
  ranges_.emplace(ServoDriver::Servos::ELBOW,
                  make_pair(make_pair(640, 1850), make_pair(0, 135)));
  ranges_.emplace(ServoDriver::Servos::WRIST,
                  make_pair(make_pair(2450, 600), make_pair(-90, 90)));
  ranges_.emplace(ServoDriver::Servos::GRIPPER,
                  make_pair(make_pair(2100, 900), make_pair(0, 1)));
  ranges_.emplace(ServoDriver::Servos::WRIST_ROTATE,
                  make_pair(make_pair(570, 1900), make_pair(-90, 90)));
  for (ServoDriver::Servos i = BASE; i < ENUM_END;
       i = ServoDriver::Servos(i + 1))
    servoMap_.emplace(i, 0);
}

int16_t degreeToPWM(Servos servo, int16_t degree) {
  int16_t pwm = 0;
  if (degree < ranges_.at(servo).second.first ||
      degree > ranges_.at(servo).second.second) {
    cout << "Degree out of range" << endl;
    return -1;
  }
  pwm = (degree - ranges_.at(servo).second.first) *
            (ranges_.at(servo).first.second - ranges_.at(servo).first.first) /
            (ranges_.at(servo).second.second - ranges_.at(servo).second.first) +
        ranges_.at(servo).first.first;
  return pwm;
}

int16_t PWMtoDegree(Servos servo, int16_t pwm) {
  int16_t degree = 0;
  if (pwm < ranges_.at(servo).first.first ||
      pwm > ranges_.at(servo).first.second) {
    cout << "PWM out of range" << endl;
    return -1;
  }
  degree =
      (pwm - ranges_.at(servo).first.first) *
          (ranges_.at(servo).second.second - ranges_.at(servo).second.first) /
          (ranges_.at(servo).first.second - ranges_.at(servo).first.first) +
      ranges_.at(servo).second.first;
  return degree;
}

void ServoDriver::setServoDegree(Servos servo, int16_t degree,
                                 int16_t time = 0) {
  int speed = 0;
  cout << "Setting servo degree..." << endl;
  uint16_t defaultSpeed = 1000;
  ostringstream servoMSG;
  int16_t pwm = degreeToPWM(servo, degree);
  if (pwm < 0) {
    return;
  }
  servoMSG << "#" << servo << "P" << pwm;
  if (!emergencyPause) {
    if (speed != 0 && time != 0) {
      cout << "speed + time given" << endl;
    } else if (speed != 0) {
      servoMSG << "S" << speed;
    } else if (speed == 0 && time != 0) {
      servoMSG << "T" << time;
    } else if (speed == 0 && time == 0) {
      servoMSG << "S" << defaultSpeed;
    }
    servoMSG << "\r";

    serialServos_.sendSerialMsg(servoMSG);
    servoMap_[servo] = degree;
  }
}

int16_t ServoDriver::getServoDegree(Servos servo) {
  return servosMap_.at(servo);
}

void ServoDriver::disengageEmergencyStop() {
  if (emergencyStopEngaged_)
    emergencyStopEngaged_ = false;
  else
    cout << "Emergency stop not engaged, doing nothing..." << endl;
}

void ServoDriver::engageEmergencyStop() {
  // Exit if already engaged:
  if (emergencyStopEngaged_)
  {
    cout << "Emergency stop already engaged, doing nothing..." << endl;
    return;
  }

  // Stop all servos:
  ostringstream servoMSG;
  for (int i = 0; i < servoCount_; i++) {
    servoMSG << "STOP" << i << "\r";
    serialServos_.sendSerialMsg(servoMSG);
  }
  emergencyPause = true;
}

int ServoDriver::getServoCount() { return servoCount_; }