#ifndef DOMUS_SERVO_H_
#define DOMUS_SERVO_H_
#include "XL320.h"
#include "SoftwareSerial.h"

class ServoMotors
{
  public:
    ServoMotor(int rx_pin, int tx_pin);
    void SetTarget();
    // we want to update servo positions with a 10millisecond offset between each.
    // because we have communication issues along the servo line.
    // in order to do that, we have the calling code call UpdateServoPositions repeatedly
    // and if there is a request in the queue, and it's been more than 10 milliseconds since the last request
    // then we're good to go.
    void UpdateServoPosition(int joint_id, float destination_angle);
  private:
    XL320 _servos;
    SoftwareSerial _servo_serial;
    int servo_to_request_next;
    int[3] servo_command_int;
};

#endif
