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

ServoDriver::ServoDriver() : servoCount_(6), emergencyStopEngaged_(false) {
    
}

ServoDriver::~ServoDriver() {}	

void ServoDriver::setServoDegree(Servos servo, int16_t degree, int16_t time = 0) {
	// do not know yet
}

int ServoDriver::getServoDegree(Servos servo)
{

}

void ServoDriver::disengageEmergencyStop() {
	emergencyStopEngaged_ = false;
	// how to send msg, I dont know yet !TODO
}

void ServoDriver::engageEmergencyStop() {
	emergencyStopEngaged_ = true;
	// how to send msg, I dont know yet !TODO
}

int ServoDriver::getServoCount() {
	return servoCount_;
}