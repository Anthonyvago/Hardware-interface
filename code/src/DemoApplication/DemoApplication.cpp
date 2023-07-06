/**
 * @file DemoApplication.cpp
 * @author Anthony Vágó (A.Vago@student.han.nl)
 * @brief
 * @version 0.1
 * @date 2023-06-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "DemoApplication/DemoApplication.hpp"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <vector>

using namespace chrono_literals;
using namespace std;

DemoApplication::DemoApplication() : Node("DemoApplication")
{
  timer_ = this->create_wall_timer(chrono::milliseconds(100), bind(&DemoApplication::timerCallback, this));
  servoPub_ = this->create_publisher<hardware_interface::msg::Setservos>("RobotArmDriverServos", 10);
  statePub_ = this->create_publisher<hardware_interface::msg::Setrobotarmstate>("RobotArmDriverState", 10);
}

// Not sure if this is the best way to split a string, but it works for now.
vector<string> split(string s, string delimiter)
{
  size_t pos_start = 0, pos_end, delim_len = delimiter.length();
  string token;
  vector<string> res;

  while ((pos_end = s.find(delimiter, pos_start)) != string::npos)
  {
    token = s.substr(pos_start, pos_end - pos_start);
    pos_start = pos_end + delim_len;
    res.push_back(token);
  }

  res.push_back(s.substr(pos_start));
  return res;
}

bool iequals(const string& a, const string& b)
{
  uint32_t sz = a.size();
  if (b.size() != sz)
    return false;
  for (unsigned int i = 0; i < sz; ++i)
    if (tolower(a[i]) != tolower(b[i]))
      return false;
  return true;
}

void DemoApplication::printOutput_UnknownInput()
{
  RCLCPP_INFO(this->get_logger(), "Sorry, but what is this for kind of command ?!");
  RCLCPP_INFO(this->get_logger(), "I'm not an AI chat bot! Give me commands like:");
  RCLCPP_INFO(this->get_logger(),
              "'set servo 0 180 2000' where 0=servo index, 180=degrees, "
              "2000=time to execute in milliseconds");
  RCLCPP_INFO(this->get_logger(), "'set state park' where park is a state");
}

void DemoApplication::timerCallback()
{
  string input = "";
  cout << "Input > ";
  getline(cin, input);
  vector<string> parsedInput = split(input, " ");

  // check if user gave the minimum number of words to create a command:
  // least number of words can create command 'set state park'
  if (parsedInput.size() < 3)
  {
    RCLCPP_INFO(this->get_logger(),
                "Give me at least 3 seperate words to form a command, like 'set servo 0 180 "
                "2000'!");
    return;  // exit function
  }

  string s = "Handling input: ";
  for (size_t i = 0; i < parsedInput.size(); i++)
    s += parsedInput[i] + " ";
  RCLCPP_INFO(this->get_logger(), s.c_str());

  if (iequals(parsedInput[0], "SET"))
  {
    if (iequals(parsedInput[1], "SERVO"))
    {
      vector<unsigned short> servos;
      vector<short> degrees;
      vector<unsigned short> times;
      for (size_t i = 2; i < parsedInput.size(); i += 3)
      {
        // quick check for possible crash:
        if (parsedInput.size() < i + 3)
        {
          RCLCPP_INFO(this->get_logger(), "Cannot execute command because too little arguments are given!");
          return;  // exit this function
        }

        servos.push_back(stoi(parsedInput[i]));
        degrees.push_back(stoi(parsedInput[i + 1]));
        times.push_back(stoi(parsedInput[i + 2]));
      }
      setServoDegrees(servos, degrees, times);
    }
    else if (iequals(parsedInput[1], "STATE"))
    {
      if (iequals(parsedInput[2], "READY"))
      {
        setRobotArmState(READY);
      }
      else if (iequals(parsedInput[2], "PARK"))
      {
        setRobotArmState(PARK);
      }
      else if (iequals(parsedInput[2], "STRAIGHT_UP"))
      {
        setRobotArmState(STRAIGHT_UP);
      }
      else if (iequals(parsedInput[2], "EMERGENCY_STOP"))
      {
        setRobotArmState(EMERGENCY_STOP);
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Sorry, I don't know this state");
      }
    }
    else
    {
      printOutput_UnknownInput();
    }
  }
  else
  {
    printOutput_UnknownInput();
  }

  cout << endl;
}

void DemoApplication::setServoDegrees(const vector<unsigned short>& servos, const vector<short>& degrees,
                                      const vector<unsigned short>& times)
{
  auto message = hardware_interface::msg::Setservos();
  message.degrees = degrees;
  message.servos = servos;
  message.times = times;
  servoPub_->publish(message);
}

void DemoApplication::setRobotArmState(RobotArmState state)
{
  auto message = hardware_interface::msg::Setrobotarmstate();
  message.state = state;
  statePub_->publish(message);
}
