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

DemoApplication::DemoApplication() : Node("DemoApplication") {
  timer_ = this->create_wall_timer(chrono::milliseconds(100),
                                   bind(&DemoApplication::timerCallback, this));
  servoPub_ = this->create_publisher<hardware_interface::msg::SetServos>(
      "RobotArmDriver", 10);
  statePub_ = this->create_publisher<hardware_interface::msg::setRobotArmState>(
      "RobotArmDriver", 10);
}

// Not sure if this is the best way to split a string, but it works for now.
vector<string> split(string s, string delimiter) {
  size_t pos_start = 0, pos_end, delim_len = delimiter.length();
  string token;
  vector<string> res;

  while ((pos_end = s.find(delimiter, pos_start)) != string::npos) {
    token = s.substr(pos_start, pos_end - pos_start);
    pos_start = pos_end + delim_len;
    res.push_back(token);
  }

  res.push_back(s.substr(pos_start));
  return res;
}

void DemoApplication::timerCallback() {
  string input = "";
  cout << "Input > ";
  getline(cin, input);
  vector<string> parsedInput = split(input, " ");

  // check if user gave the minimum number of words to create a command:
  // least number of words can create command 'set state park'
  if (parsedInput.size() < 3) {
    cout << "Give me at least 3 seperate words to form a command, like "
            "'set servo 0 180 2000'!"
         << endl;
    return; // exit function
  }

  if (strcmp(parsedInput[0], "SET") == 0 ||
      strcmp(parsedInput[0], "set") == 0) {
    if (strcmp(parsedInput[1], "SERVO") == 0 ||
        strcmp(parsedInput[1], "servo") == 0) {
      vector<unsigned short> servos;
      vector<short> degrees;
      vector<unsigned short> times;
      for (size_t i = 2; i < parsedInput.size(); i += 3) {
        // quick check for possible crash:
        if (parsedInput.size() < i + 3) {
          cout << "Cannot execute command because too little arguments "
                  "are given!"
               << endl;
          return; // exit this function
        }

        servos.push_back(stoi(parsedInput[i]));
        degrees.push_back(stoi(parsedInput[i + 1]));
        times.push_back(stoi(parsedInput[i + 2]));
      }
      setServoDegrees(servos, degrees, times);
    }
  } else if (strcmp(parsedInput[1], "STATE") == 0 ||
             strcmp(parsedInput[1], "state") == 0) {
    if (strcmp(parsedInput[2], "READY") == 0 ||
        strcmp(parsedInput[2], "ready") == 0) {
      setRobotArmState(READY);
    } else if (strcmp(parsedInput[2], "PARK") == 0 ||
               strcmp(parsedInput[2], "park") == 0) {
      setRobotArmState(PARK);
    } else if (strcmp(parsedInput[2], "STRAIGHT_UP") == 0 ||
               strcmp(parsedInput[2], "straight_up") == 0) {
      setRobotArmState(STRAIGHT_UP);
    } else if (strcmp(parsedInput[2], "EMERGENCY_STOP") == 0 ||
               strcmp(parsedInput[2], "emergency_stop") == 0) {
      setRobotArmState(EMERGENCY_STOP);
    } else {
      cout << "Sorry, I don't know this state" << endl;
    }
  } else {
    cout << "Sorry, but what is this for kind of command ?!" << endl;
    cout << "I'm not an AI chat bot! Give me commands like:" << endl;
    cout << "'set servo 0 180 2000' where 0=servo index, 180=degrees, "
            "2000=time to execute in milliseconds"
         << endl;
    cout << "'set state park'" << endl;
  }

  cout << endl;
}

void DemoApplication::setServoDegrees(const vector<unsigned short> &servos,
                                      const vector<short> &degrees,
                                      const vector<unsigned short> &times) {
  auto message = hardware_interface::msg::SetServos();
  message.degrees = degrees;
  message.servos = servos;
  message.times = times;
  servoPub_->publish(message);
}

void DemoApplication::setRobotArmState(RobotArmState state) {
  auto message = hardware_interface::msg::setRobotArmState();
  message.state = state;
  statePub_->publish(message);
}