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

        ServoDriver();

        ~ServoDriver();	

        void setServoDegree(Servos servo, int16_t degree, int16_t time = 0);

        int getServoDegree(Servos servo);

        void disengageEmergencyStop();

        void engageEmergencyStop();

        int getServoCount();

    private:
        int servoCount_;

        bool emergencyStopEngaged_;

};

#endif // SERVODRIVER_HPP