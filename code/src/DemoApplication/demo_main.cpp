/**
 * @file demo_main.cpp
 * @author Anthony Vágó (A.Vago@student.han.nl)
 * @brief 
 * @version 0.1
 * @date 2023-06-12
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <DemoApplication/DemoApplication.hpp>
#include <iostream>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DemoApplication>());
  rclcpp::shutdown();
  return 0;
}