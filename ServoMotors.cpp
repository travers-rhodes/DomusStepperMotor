#include "ServoMotors.h"

ServoMotors::ServoMotors(int rx_pin, int tx_pin) : _servo_serial(rx_pin, tx_pin)
{
  _last_update_time_millis = millis();
}

// initialize connection
void ServoMotors::Initialize()
{
  _servo_serial.begin(115200);
  _servos.begin(_servo_serial);
  for (int i = 1; i <= 3; i++){
    delay(10);
    _servos.setJointSpeed(i, 100);
  }
}

void ServoMotors::SetFifthJoint(float angle)
{
  // each unit is 0.29 degrees
  // straight up and down is 512
  // first, compute delta off of 0
  // then add/subtract from the two aligned motors
  float float_delta = angle / (2 * PI) * 360.0 / (0.29);
  int int_delta = (int) float_delta;
  // the plus/minus below makes us line up with the default URDF provided by niryo
  AddServoRequestToQueue(1, 512 + int_delta);
  AddServoRequestToQueue(3, 512 - int_delta);
}


void ServoMotors::SetSixthJoint(float angle)
{
  // each unit is 0.29 degrees
  // straight up and down is 512
  float float_command = angle / (2 * PI) * 360.0 / (0.29) + 512;
  int int_command = (int) float_command;
  AddServoRequestToQueue(2, int_command);
}

// we want to update servo positions with a 10millisecond offset between each
// because we have communication issues along the servo line.
// in order to do that, we have the calling code call UpdateServoPositions repeatedly
// and if there is a request in the queue, and it's been more than 10 milliseconds since the last request
// then we're good to go.
void ServoMotors::UpdateServoPositions()
{
  // do nothing if no command
  if (_command_queue.size() == 0)
  {
    return;
  }

  // if we've waited more than 10 millis, then kick off the next
  // servo command
  unsigned long cur_millis = millis();
  if (cur_millis - _last_update_time_millis > 10
        || (cur_millis < _last_update_time_millis
            && cur_millis > 10))
  {
    _last_update_time_millis = cur_millis;
    ServoCommand command = _command_queue.pop();
    _servos.moveJoint(command.servo_id, command.value);
  }
}

// pop the servo request onto the queue
void ServoMotors::AddServoRequestToQueue(int servo_id, int value)
{
  ServoCommand command;
  command.servo_id = servo_id;
  command.value = value;
  _command_queue.push(command);  
}
