#include "StepperMotor.h"
#include "Arduino.h"

// the fastest we send ticks to motor
const int MIN_TICK_PERIOD_MICROS = 400;
const int PULSE_LENGTH_MICROS = 200;

StepperMotor::StepperMotor()
{
  //default constructor does nothing. you shouldn't use this object constructed this way, but I'm new to cpp...
}

StepperMotor::StepperMotor(int ticks_per_revolution, float gear_ratio, float min_angle, float max_angle, int step_pin, int direction_pin)
{
  _ticks_per_revolution = ticks_per_revolution;
  _gear_ratio = gear_ratio;
  _min_bound_in_ticks = ConvertAngleToTicks(min_angle);
  _max_bound_in_ticks = ConvertAngleToTicks(max_angle);
  _step_pin = step_pin;
  _direction_pin = direction_pin;
  _is_destination_active = false; 
  _motor_direction = -1;
}

void StepperMotor::Calibrate(float current_angle)
{
  _is_destination_active = false;
  _current_state_in_ticks = ConvertAngleToTicks(current_angle);
}

void StepperMotor::SetTarget(unsigned long destination_time_millis, float destination_angle)
{
  _is_destination_active = false;
  _destination_time_millis = destination_time_millis;
  _destination_in_ticks = ConvertAngleToTicks(destination_angle);
  //Serial.print(_destination_in_ticks);
  _last_motor_uptick_micros = micros();
  // set the motor direction as needed
  int abs_ticks_to_move;
  if (_current_state_in_ticks == _destination_in_ticks)
  {
    // we're already there, so no need to move
    return;
  }
  
  if (_current_state_in_ticks >= _destination_in_ticks)
  {
    if (_motor_direction != -1) {
      digitalWrite(_direction_pin, HIGH);
      _motor_direction = -1;
    }
    //Serial.print("going right");
    abs_ticks_to_move = _current_state_in_ticks - _destination_in_ticks;
  }
  else
  {
    if (_motor_direction != 1) {
      digitalWrite(_direction_pin, LOW);
      _motor_direction = 1;
    }
    //Serial.print("going left");
    abs_ticks_to_move = _destination_in_ticks - _current_state_in_ticks;
  } 
  unsigned long current_time_millis = millis();
  if (current_time_millis >= _destination_time_millis)
  {
    //Serial.print("we can't go back in time");
    return;
  }
   
  _motor_tick_period_micros = (unsigned long)((_destination_time_millis - current_time_millis) * 1000)/abs_ticks_to_move;
  if (_motor_tick_period_micros < MIN_TICK_PERIOD_MICROS) {
    _motor_tick_period_micros = MIN_TICK_PERIOD_MICROS;    
  }
  //Serial.print(_motor_tick_period_micros);
  _is_destination_active = true;
  //Serial.print(_is_destination_active);
}

void StepperMotor::UpdatePosition()
{
  if (!_is_destination_active) {
    // if we're not trying to go anywhere, don't do anything
    return;
  }
  // we've taken long enough, so stop moving
  //if (millis() > _destination_time_millis)
  //{
  //  _is_destination_active = false;
  //}
  if (_current_state_in_ticks >= _destination_in_ticks)
  {
    _is_destination_active = false;
  }
  if (_is_stepper_signal_high && (micros() > _last_motor_uptick_micros + PULSE_LENGTH_MICROS))
  {
    digitalWrite(_step_pin, LOW);
    //Serial.print("Going low\n");
    _is_stepper_signal_high = false;
    return;
  }
  if (!_is_stepper_signal_high && micros() > _last_motor_uptick_micros + _motor_tick_period_micros)
  {
    digitalWrite(_step_pin, HIGH);
    //Serial.print("Going high\n");
    _is_stepper_signal_high = true;
    _last_motor_uptick_micros = micros();
    //update the current state
    _current_state_in_ticks += _motor_direction;
  }
}

int StepperMotor::ConvertAngleToTicks(float angle)
{
  return(angle / (2 * PI) * _gear_ratio * _ticks_per_revolution);
}
