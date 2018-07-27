// I know this is silly, but Arduino doesn't include std::queue, and I don't think I need all the work
// they put in to QueueArray (including blinking on error, etc)
// How to build a QueueArray is defined in https://playground.arduino.cc/Code/QueueArray
// and this code is primarily inspired from that

#ifndef DOMUS_QUEUE_H_
#define DOMUS_QUEUE_H_
#include "Arduino.h"

struct ServoCommand
{
  int servo_id;
  int value;
};


class ServoCommandQueue
{
  public:
    ServoCommandQueue();
    void push(const ServoCommand i);
    ServoCommand pop();
    int size();
 
  private:
    int _head;
    int _tail;
    int _num_items;

    // for now, for convenience, we hard-code the queue size
    ServoCommand _queue_array[10]; 
    int _size = 10;

    void blink() const; // our simple error method
};
#endif
