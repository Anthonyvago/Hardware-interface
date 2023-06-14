/**
 * @file robotarm_main.cpp
 * @author Anthony Vágó (A.Vago@student.han.nl)
 * @brief 
 * @version 0.1
 * @date 2023-06-14
 * 
 * @copyright Copyright (c) 2023
 * 
 */

// All includes are in the following include:
#include <Robot_arm_driver/RobotArmDriver.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<RobotArmDriver>());
    rclcpp::shutdown();

    return 0;
}

