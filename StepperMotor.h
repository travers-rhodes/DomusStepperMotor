#ifndef DOMUS_MOTOR_H_
#define DOMUS_MOTOR_H_

class StepperMotor
{
  public:
    StepperMotor();
    StepperMotor(int ticks_per_revolution, float gear_ratio, float min_angle, float max_angle, int step_pin, int direction_pin);
    void Calibrate(float current_angle);
    // set the target for this motor.
    // the target has two components:
    // the desired destination angle and the time we should get there
    // the time corresponds to the time that millis() would return on the arduino
    void SetTarget(unsigned long destination_time_millis, float destination_angle);
    // Check if it's time to take a motor tick, and if so, take a tick
    void UpdatePosition();
    // update the params that define the motor speed
    void SetMotorSpeed();
  private:
    int ConvertAngleToTicks(float angle);
    //
    // Constants related to robot motor mechanics
    // 
    // the number of ticks needed to rotate the motor one revolution.
    // This is a characteristic of the motor driver
    int _ticks_per_revolution;
    float _gear_ratio;
    int _min_bound_in_ticks;
    int _max_bound_in_ticks;
    //
    // variables related to user request to move motor
    //
    // when to arrive at destination (in millis since start of arduino)
    unsigned long _destination_time_millis;
    // when the _destination_time_millis attribute was last set
    unsigned long _destination_creation_millis;
    // target destination (in motor ticks. 0 is ref config)
    int _destination_in_ticks;
    // Whether or not we are actively pursuing a course to the destination or not
    // This can be set to false in order to immediately stop the motor (should also be false during calibration)
    // Setting this to false is basically like setting _destination_in_ticks to NULL
    bool _is_destination_active;
    //
    // variables related to intermediate target
    // we use an intermediate target in order to smooth
    // out target changes
    //
    // how long to update intermediate target to new target
    int _direction_change_time_millis;
    // intermediate target in ticks
    int _intermediate_ticks;
    // intermediate target time
    unsigned long _intermediate_time_millis;
    int _previous_destination_in_ticks;
    unsigned long _previous_destination_time_millis;
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
