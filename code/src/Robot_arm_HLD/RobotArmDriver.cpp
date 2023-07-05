/**
 * @file RobotArmDriver.cpp
 * @author Anthony Vágó (A.Vago@student.han.nl)
 * @brief
 * @version 0.1
 * @date 2023-06-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <rclcpp/rclcpp.hpp>

#include "Robot_arm_HLD/RobotArmDriver.hpp"
#include "Servo_LLD/ServoDriver.hpp"

using namespace std;

// "servoDriver_()" must be
// "servoDriver_("/dev/ttyUSB0")" in the future
RobotArmDriver::RobotArmDriver()
    : Node("RobotArmDriver"), servoDriver_("/dev/ttyUSB0"), curState_(PARK),
      servoActivationTimes_(servoDriver_.getServoCount(), 0)
{

  timer_ =
      this->create_wall_timer(100ms, bind(&RobotArmDriver::runQueue, this));
  servoSub_ = this->create_subscription<hardware_interface::msg::Setservos>(
      "RobotArmDriverServos", 10, bind(&RobotArmDriver::setServos, this, placeholders::_1));
  stateSub_ =
      this->create_subscription<hardware_interface::msg::Setrobotarmstate>(
          "RobotArmDriverState", 10, bind(&RobotArmDriver::setRobotArmState, this, placeholders::_1));

  // Set the initial robot arm position:
  vector<uint16_t> servos = {0, 1, 2, 3, 4, 5};
  vector<int16_t> degrees = {0, -30, 135, 42, 0, 0};
  vector<uint16_t> times = {2000, 2000, 2000, 2000, 2000, 2000};

  // Convert shared pointer from servoEvent(struct) to shared pointer from Event
  // (struct):
  shared_ptr<EventServo> servoEvent(new EventServo(SERVO, servos, degrees, times));
  eventsQueue_.push(dynamic_pointer_cast<Event>(servoEvent));
}

RobotArmDriver::~RobotArmDriver() {}

uint64_t calcServoDuration(int16_t newPos, int16_t oldPos)
{
  int16_t diff = abs(newPos - oldPos);
  double speedDegreeMS = 3.5; // 3.5ms per degree
  return (uint64_t)round((double)diff * speedDegreeMS);
}

float toRadians(float degrees) { return degrees * (M_PI / 180.0f); }

float toDegrees(float radians) { return radians * (180.0f / M_PI); }

bool isSafe(float shoulderPos, float elbowPos, float wristPos)
{
  float lijn_G = 85.725f;
  float lijn_F = 107.95f;
  float lijn_P = 95.25f;
  float theta_1 = shoulderPos;
  float theta_2 = elbowPos;
  float theta_3 = wristPos;
  float hoek_A2 = 90 - theta_1;
  float hoek_B3 = 180 - hoek_A2;
  float hoek_B2 = 180 - theta_2;
  float lijn_C = lijn_F * cosf(toRadians(hoek_B3 + hoek_B2));
  float hoek_C1 = toDegrees(asinf(lijn_C / lijn_F));
  float hoek_C3 = 180 - 90 - hoek_C1 - theta_3;
  float result = ((lijn_P * tanf(toRadians(hoek_A2))) -
                  (lijn_F * cosf(toRadians(hoek_B3 + hoek_B2)))) -
                 (lijn_G * cosf(toRadians(hoek_C3)));
  result = abs(result);
  return result > 0.0f;
}

void RobotArmDriver::setServos(
    const hardware_interface::msg::Setservos::SharedPtr msg)
{
  if (curState_ == EMERGENCY_STOP)
  {
    RCLCPP_WARN(this->get_logger(),
                "Cannot change state while current state is emergency stop!");
    return;
  }

  vector<uint16_t> servoVector = msg->servos;
  vector<int16_t> degreeVector = msg->degrees;
  vector<uint16_t> timesVector = msg->times;
  // Convert shared pointer from servoEvent(struct) to shared pointer from
  // Event (struct):
  shared_ptr<EventServo> servoEvent(new EventServo(SERVO, servoVector, degreeVector, timesVector));

  // Add to queue:
  eventsQueue_.push(dynamic_pointer_cast<Event>(servoEvent));
}

void RobotArmDriver::setRobotArmState(
    const hardware_interface::msg::Setrobotarmstate::SharedPtr msg)
{
  RobotArmState newState = static_cast<RobotArmState>(msg->state);

  // Check if the "old" state was the emegency stop:
  if (curState_ == EMERGENCY_STOP)
  {
    // Check if the new state is the emegency stop, so robot arm is still in the
    // emergency state:
    if (newState == EMERGENCY_STOP)
    {
      RCLCPP_INFO(this->get_logger(), "Emergency stop was already engaged!");
    }
    // Check if the new state is not the emegency stop, so robot arm is exiting
    // the emegency stop state:
    else
    {
      RCLCPP_INFO(this->get_logger(), "Disengaging emergency stop!");
      servoDriver_.disengageEmergencyStop();
    }
  }

  switch (newState)
  {
    case READY:
    {
      RCLCPP_INFO(this->get_logger(), "Going into READY state...");

      // Set READY positions:
      vector<uint16_t> servos{0, 1, 2, 3, 4, 5};
      vector<int16_t> degrees{0, -30, 135, 0, 0, 0};
      vector<uint16_t> times{2000, 2000, 2000, 2000, 2000, 2000};

      // Convert shared pointer from servoEvent(struct) to shared pointer from
      // Event (struct):
      shared_ptr<EventServo> servoEvent(new EventServo(SERVO, servos, degrees, times));

      // Add to queue:
      eventsQueue_.push(dynamic_pointer_cast<Event>(servoEvent));
      break;
    }
    case PARK:
    {
      RCLCPP_INFO(this->get_logger(), "Going into PARK state...");

      // Set PARK positions:
      vector<uint16_t> servos{0, 1, 2, 3, 4, 5};
      vector<int16_t> degrees{0, -30, 135, 42, 0, 0};
      vector<uint16_t> times{2000, 2000, 2000, 2000, 2000, 2000};

      // Convert shared pointer from servoEvent(struct) to shared pointer from
      // Event (struct):
      shared_ptr<EventServo> servoEvent(
          new EventServo(SERVO, servos, degrees, times));

      // Add to queue:
      eventsQueue_.push(dynamic_pointer_cast<Event>(servoEvent));
      break;
    }
    case STRAIGHT_UP:
    {
      RCLCPP_INFO(this->get_logger(), "Going into STRAIGHT_UP state...");

      // Set STRAIGHT_UP positions:
      vector<uint16_t> servos{0, 1, 2, 3, 4, 5};
      vector<int16_t> degrees{0, 0, 0, 0, 0, 0};
      vector<uint16_t> times{2000, 2000, 2000, 2000, 2000, 2000};

      // Convert shared pointer from servoEvent(struct) to shared pointer from
      // Event (struct):
      shared_ptr<EventServo> servoEvent(
          new EventServo(SERVO, servos, degrees, times));

      // Add to queue:
      eventsQueue_.push(dynamic_pointer_cast<Event>(servoEvent));
      break;
    }
    case EMERGENCY_STOP:
    {
      RCLCPP_INFO(this->get_logger(), "Going into EMERGENCY_STOP state...");

      // Engage the emergency stop:
      servoDriver_.engageEmergencyStop();

      // Clear the queue by replacing it with an empty queue (most efficient way
      // according to StackOverflow):
      queue<shared_ptr<Event>> empty;
      swap(eventsQueue_, empty);
      break;
    }
  }
}

void RobotArmDriver::runQueue()
{
  if (!eventsQueue_.empty())
  {
    // Before running the queue, check if all the servos are ready to move.
    // We do this by checking if the servo with the highest activation timer
    // (last servo to move) already done moving.

    uint64_t curTime =
        chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch())
            .count();
    uint64_t lastMovingServoFinishingTime =
        *max_element(servoActivationTimes_.begin(), servoActivationTimes_.end());
    if (curTime > lastMovingServoFinishingTime)
    {
      // The last moving servo is done moving, so we can run the queue:

      // Get the first event in the queue:
      shared_ptr<Event> event = eventsQueue_.front();

      switch (event->eventType)
      {
      case SERVO:
      {
        // Cast the event to a servo event to retreive the servo data:
        shared_ptr<EventServo> servoEvent =
            dynamic_pointer_cast<EventServo>(event);

        // Retrieve the future positions from each servo to determine if it is
        // safe to move:
        int16_t shoulderPos =
            servoDriver_.getServoDegrees(Servos::SHOULDER);
        int16_t elbowPos =
            servoDriver_.getServoDegrees(Servos::ELBOW);
        int16_t wristPos =
            servoDriver_.getServoDegrees(Servos::WRIST);
        for (size_t i = 0; i < servoEvent->servos.size(); i++)
        {
          switch (servoEvent->servos[i])
          {
          case 1:
            shoulderPos = servoEvent->degrees[i];
            break;
          case 2:
            elbowPos = servoEvent->degrees[i];
            break;
          case 3:
            wristPos = servoEvent->degrees[i];
            break;
          }
        }

        // Determine if it is safe to move:
        if (isSafe(shoulderPos, elbowPos, wristPos))
        {
          // It is safe to move, so we can move the servos:
          for (unsigned int i = 0; i < servoEvent->servos.size(); i++)
          {
            // Calculate the minimum time to move the servo:
            uint64_t minTime = calcServoDuration(
                servoEvent->degrees[i],
                servoDriver_.getServoDegrees(
                    (Servos)servoEvent->servos[i]));
            if (servoEvent->times[i] > minTime)
            {
              minTime = servoEvent->times[i];
            }
            servoActivationTimes_[i] = curTime + minTime;
            string output = "Moving servo: " + to_string(servoEvent->servos[i]) +
                            " to " + to_string(servoEvent->degrees[i]) +
                            " degrees in " + to_string(minTime) +
                            " milliseconds.";
            RCLCPP_INFO(this->get_logger(), output.c_str());
            servoDriver_.setServoDegrees(
                (Servos)servoEvent->servos[i],
                servoEvent->degrees[i], minTime);
          }
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(),
                       "One or more servo positions are not in a safe range!");
        }

        break;
      }
      case STATE:
      {
        shared_ptr<EventState> stateEvent =
            dynamic_pointer_cast<EventState>(event);
        curState_ = stateEvent->desiredState;
        break;
      }

        // Remove the handled event from the queue:
        eventsQueue_.pop();
      }
    }
    else
    {
      // The last moving servo is not done moving, so we wait until it is done
      // moving.
      string output = "Waiting " + to_string(lastMovingServoFinishingTime - curTime) +
              "  milliseconds for the last moving servo to finish moving...";
      RCLCPP_INFO(this->get_logger(), output.c_str());
    }
  }
}
