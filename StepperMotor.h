#ifndef DOMUS_MOTOR_H_
#define DOMUS_MOTOR_H_
#include "Arduino.h"

class StepperMotor
{
  public:
    StepperMotor();
    StepperMotor(int ticks_per_revolution, 
      float gear_ratio, 
      int step_pin, 
      int direction_pin,
      double max_motor_speed_ticks_per_micro,
      double motor_acceleration_ticks_per_micro_squared);
    void Calibrate(float current_angle);
    // set the target for this motor.
    // the target has two components:
    // the desired destination angle and the time we should get there
    // the time corresponds to the time that millis() would return on the arduino
    void SetTarget(unsigned long destination_time_millis, float destination_angle);
    void UpdateSpeed();
    // Check if it's time to take a motor tick, and if so, take a tick
    void UpdatePosition();
  private:
    void UpdateMotorDirectionPin();
    int ConvertAngleToTicks(float angle);

    //
    // Constants related to robot motor mechanics
    // 
    // the number of ticks needed to rotate the motor one revolution.
    // This is a characteristic of the motor driver
    int _ticks_per_revolution;
    float _gear_ratio;

    //
    // variables related to user request to move motor
    //
    // target destination (in motor ticks. 0 is ref config)
    // only guaranteed to exist if _is_destination_active
    int _destination_in_ticks;
    // Whether or not we are actively pursuing a course to the destination or not
    // This can be set to false in order to immediately stop the motor (should also be false during calibration)
    // Setting this to false is basically like setting _destination_in_ticks to NULL
    bool _is_destination_active;

    //
    // variables related to current motor state
    //
    // Current position of the motor (in motor ticks. 0 is ref config)
    int _current_state_in_ticks;
    //
    // last time motor moved (in micros since start of arduino). 
    // micros() overflows every 70 minutes, but like... that's a long time, right?
    unsigned long _last_motor_uptick_micros;

    //
    // Variables related to commands to move motor
    //
    // which direction we want to be spinning the motor (ie: left or right)
    int _motor_direction;
    // this number should be probably no more than around
    // 1 tick / 200 micros
    double _max_motor_speed_ticks_per_micro;
    // who knows what order of magnitude this number should be, but it
    // says how much we let the robot arm accelerate.
    double _motor_acceleration_ticks_per_micro_squared;
    // (a very small double. The inverse of _motor_tick_period_micros)
    double _current_speed_ticks_per_micro;
    // micros clock time when we last updated the motor's speed
    // needed in finite difference updating for acceleration
    unsigned long _time_of_last_speed_update_micros;
    // (a very small double)
    // this is the naive speed the arm would want to go if there were no smoothing.
    double _target_speed_ticks_per_micro;
    // how many micros between motor ticks in order to reach destination on time
    unsigned long _motor_tick_period_micros;
    // are we currently sending a high (or low) pulse to the stepper motor
    bool _is_stepper_signal_high;

    //
    // Arduino communication pins
    //
    int _step_pin;
    int _direction_pin;
};

#endif
