/**
 * @file SerialServoCommunication.cpp
 * @author Anthony Vágó (A.Vago@student.han.nl)
 * @brief
 * @version 0.1
 * @date 2023-06-13
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "Servo_LLD/SerialServoCommunication.hpp"

using namespace std;

SerialServoCommunication::SerialServoCommunication(const string &pathToDevice)
    : ioService_(), serial_(ioService_, pathToDevice) {
  serial_.set_option(boost::asio::serial_port_base::baud_rate(9600));
  serial_.set_option(boost::asio::serial_port::flow_control(
      boost::asio::serial_port::flow_control::none));
  serial_.set_option(
      boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
  serial_.set_option(boost::asio::serial_port::stop_bits(
      boost::asio::serial_port::stop_bits::one));
  serial_.set_option(boost::asio::serial_port::character_size(
      boost::asio::serial_port::character_size(8)));
}

void SerialServoCommunication::sendSerialMsg(const string &serialMsg) {
  boost::asio::streambuf streamBuff;
  ostringstream ss;
  ostream os(&streamBuff);
  os << serialMsg << "\n" << "\r";
  boost::asio::write(serial_, streamBuff.data());
  os.flush();
}

void SerialServoCommunication::sendSerialMsg(const ostringstream &serialMsg) {
  boost::asio::streambuf streamBuff;
  ostringstream ss;
  ostream os(&streamBuff);
  os << serialMsg.str() << "\n" << "\r";
  boost::asio::write(serial_, streamBuff.data());
  os.flush();
}