#ifndef DOMUS_SERVO_H_
#define DOMUS_SERVO_H_
#include "XL320.h"
#include "SoftwareSerial.h"
#include "ServoCommandQueue.h" // Arduino doesn't natively support std::queue

class ServoMotors
{
  public:
    ServoMotors(int rx_pin, int tx_pin);
    void SetFifthJoint(float angle);
    void SetSixthJoint(float angle);
    // we want to update servo positions with a 10millisecond offset between each
    // because we have communication issues along the servo line.
    // in order to do that, we have the calling code call UpdateServoPositions repeatedly
    // and if there is a request in the queue, and it's been more than 10 milliseconds since the last request
    // then we're good to go.
    void UpdateServoPositions();
  private:
    XL320 _servos;
    SoftwareSerial _servo_serial;
    unsigned long _last_update_time_millis;
    ServoCommandQueue _command_queue;

    void AddServoRequestToQueue(int servo_id, int value);
};

#endif
