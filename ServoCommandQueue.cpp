#include "ServoCommandQueue.h"

ServoCommandQueue::ServoCommandQueue()
{
  _head = 0;
  _tail = 0;
  _num_items = 0;
}

void ServoCommandQueue::push(const ServoCommand i)
{
  _queue_array[_tail++] = i;
  if (_tail = _size)
  {
    _tail = 0;
  }
  _num_items++;
  if (_num_items == _size) {
    blink();  
  }
}

ServoCommand ServoCommandQueue::pop()
{
  if (_num_items == 0) {
    blink();  
  }
  ServoCommand return_val = _queue_array[_head++];
  if (_head == _size)
  {
    _head = 0;
  }
  _num_items--;
}

int ServoCommandQueue::size()
{
  return _num_items;
}

// led blinking method in case of error.
// this method is copied directly from https://playground.arduino.cc/Code/QueueArray
void ServoCommandQueue::blink () const {
  // set led pin as output.
  pinMode (LED_BUILTIN, OUTPUT);

  // continue looping until hardware reset.
  while (true) {
    digitalWrite (LED_BUILTIN, HIGH); // sets the LED on.
    delay (250);                 // pauses 1/4 of second.
    digitalWrite (LED_BUILTIN, LOW);  // sets the LED off.
    delay (250);                 // pauses 1/4 of second.
  }

  // solution selected due to lack of exit() and assert().
}
