#include "StepperMotor.h"
#include "Arduino.h"

const int PULSE_LENGTH_MICROS = 200;

//
// Helper Methods
//
// returns speed in ticks per micro
double GetTargetSpeed(int current_state_in_ticks,
  int destination_in_ticks, 
  unsigned long current_time_millis, 
  unsigned long destination_time_millis,
  double max_motor_speed_ticks_per_micro)
{
  if (current_state_in_ticks == destination_in_ticks)
  {
    // we're already there, so no need to move
    return 0;
  }
  double ticks_to_move = destination_in_ticks - current_state_in_ticks;
  unsigned long time_remaining_millis = destination_time_millis - current_time_millis;
  // compute the naive speed, and then truncate as needed 
  // based on robot speed limits
  double speed = (ticks_to_move / time_remaining_millis) / 1000.0;

  if (abs(speed) > max_motor_speed_ticks_per_micro
       || destination_time_millis < current_time_millis)
  {
    if (ticks_to_move > 0)
    {
      speed = max_motor_speed_ticks_per_micro;
    }
    else
    {
      speed = -max_motor_speed_ticks_per_micro;
    }
  }

  return speed;
}

// get the new speed based on current speed, target speed, and 
// acceleration capabilities
double GetNewSpeed(double current_speed_ticks_per_micro,
      double target_speed_ticks_per_micro,
      double motor_acceleration_ticks_per_micro_squared,
      unsigned long micros_since_last_speed_update)
{
  // check how much we can accelerate in the alloted time
  double allowed_speed_change = motor_acceleration_ticks_per_micro_squared * micros_since_last_speed_update;
  // if we can accelerate to the target speed, do that
  if (abs(target_speed_ticks_per_micro - current_speed_ticks_per_micro) < allowed_speed_change)
  {
    return target_speed_ticks_per_micro;
  }
  // otherwise, do max acceleration in the required direction
  if (target_speed_ticks_per_micro > current_speed_ticks_per_micro)
  {
    return current_speed_ticks_per_micro + allowed_speed_change;
  }
  return current_speed_ticks_per_micro - allowed_speed_change;
}
    

//
// Class Definition
//

StepperMotor::StepperMotor()
{
  //default constructor does nothing. you shouldn't use this object constructed this way, but I'm new to cpp...
}

StepperMotor::StepperMotor(int ticks_per_revolution, 
    float gear_ratio, 
    int step_pin, 
    int direction_pin, 
    double max_motor_speed_ticks_per_micro, 
    double motor_acceleration_ticks_per_micro_squared)
{
  _ticks_per_revolution = ticks_per_revolution;
  _gear_ratio = gear_ratio;
  _step_pin = step_pin;
  _direction_pin = direction_pin;
  _max_motor_speed_ticks_per_micro = max_motor_speed_ticks_per_micro;
  _motor_acceleration_ticks_per_micro_squared = motor_acceleration_ticks_per_micro_squared;

  _is_destination_active = false; 
  _motor_direction = -1;
  _current_speed_ticks_per_micro = 0;
}

void StepperMotor::Calibrate(float current_angle)
{
  _is_destination_active = false;
  _current_state_in_ticks = ConvertAngleToTicks(current_angle);
}

void StepperMotor::SetTarget(unsigned long destination_time_millis, float destination_angle)
{
  _is_destination_active = false;
  _destination_in_ticks = ConvertAngleToTicks(destination_angle);
  _time_of_last_speed_update_micros = micros();
  
  unsigned long current_time_millis = millis();

  _target_speed_ticks_per_micro =
    GetTargetSpeed(_current_state_in_ticks,
      _destination_in_ticks,
      current_time_millis,
      destination_time_millis,
      _max_motor_speed_ticks_per_micro);

  _is_destination_active = true;
}

// update the current speed of the motor
void StepperMotor::UpdateSpeed()
{
  // nothing to do if the motor is off
  if (!_is_destination_active)
  {
    return;
  }
 
  unsigned long cur_micros = micros();
  unsigned long micros_since_last_speed_update =  
    cur_micros -  _time_of_last_speed_update_micros;

  // we need checks like this in case micros overflowed
  if (micros_since_last_speed_update < 0)
  {
    micros_since_last_speed_update = 0;
  }
  
  _current_speed_ticks_per_micro = 
    GetNewSpeed(_current_speed_ticks_per_micro,
      _target_speed_ticks_per_micro,
      _motor_acceleration_ticks_per_micro_squared,
      micros_since_last_speed_update);

  // if you're trying to go slower than 1 tick per 1000 seconds,
  // we'll have you go at one tick per 1000 seconds.
  if (abs(_current_speed_ticks_per_micro) < 1e-9) 
  {
    _motor_tick_period_micros = 1e9;
  }
  else
  {
    _motor_tick_period_micros = abs(1.0/_current_speed_ticks_per_micro);
  }
  
  //Serial.print(_target_speed_ticks_per_micro * 1e3);
  //Serial.print(" is target ");
  //Serial.print(_current_state_in_ticks);
  //Serial.print(" is position\n");

  UpdateMotorDirectionPin();

  _time_of_last_speed_update_micros = cur_micros;
}

void StepperMotor::UpdatePosition()
{
  if (!_is_destination_active) {
    // if we're not trying to go anywhere, don't do anything
    return;
  }
  if (_current_state_in_ticks == _destination_in_ticks)
  {
    // turn off the motor 
    // and therefore also log that the current speed is 0
    _is_destination_active = false;
    _current_speed_ticks_per_micro = 0;
    return;
  }

  unsigned long cur_micros = micros();
  if (_is_stepper_signal_high 
        && (cur_micros > _last_motor_uptick_micros + PULSE_LENGTH_MICROS 
            || cur_micros < _last_motor_uptick_micros))
  {
    digitalWrite(_step_pin, LOW);
    //Serial.print("Going low\n");
    _is_stepper_signal_high = false;
    return;
  }
  if (!_is_stepper_signal_high 
        && (cur_micros > _last_motor_uptick_micros + _motor_tick_period_micros
            || cur_micros < _last_motor_uptick_micros))
  {
    digitalWrite(_step_pin, HIGH);
    //Serial.print("Going high\n");
    _is_stepper_signal_high = true;
    _last_motor_uptick_micros = cur_micros;
    //update the current state
    _current_state_in_ticks += _motor_direction;
  }
}

//
// Private methods
//


// update the motor direction pin if needed
// this function only changes the pin if
// _current_speed_ticks_per_micro is requesting
// a different direction from _motor_direction
void StepperMotor::UpdateMotorDirectionPin()
{
  if (_current_speed_ticks_per_micro > 0
        && _motor_direction != 1)
  {
      digitalWrite(_direction_pin, LOW);
      _motor_direction = 1;
  }
  else if (_current_speed_ticks_per_micro < 0
             && _motor_direction != -1)
  {
      digitalWrite(_direction_pin, HIGH);
      _motor_direction = -1;
  }
}

int StepperMotor::ConvertAngleToTicks(float angle)
{
  return(angle / (2 * PI) * _gear_ratio * _ticks_per_revolution);
}
