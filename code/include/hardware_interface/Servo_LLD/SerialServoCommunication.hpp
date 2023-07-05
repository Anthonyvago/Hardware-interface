/**
 * @file SerialServoCommunication.hpp
 * @author Anthony Vágó (A.Vago@student.han.nl)
 * @brief
 * @version 0.1
 * @date 2023-06-13
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <boost/asio.hpp>
#include <iostream>

using namespace std;

class SerialServoCommunication {
public:
  SerialServoCommunication(const string &pathToDevice);
  
  SerialServoCommunication();

  ~SerialServoCommunication() = default;

  void sendSerialMsg(const string &serialMsg);

  void sendSerialMsg(const ostringstream &serialMsg);

private:
  boost::asio::io_service ioService_;
  boost::asio::serial_port serial_;
};
