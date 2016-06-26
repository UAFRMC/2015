#ifndef ENCODER_H
#define ENCODER_H

#include "milli.h"
#include "Arduino.h"

extern milli_t milli;

/**
  Read an optical encoder value, and reconstruct the
  speed of the changes being seen.
*/
class encoder_t{
  public:
   int pin; // Arduino analog pin to read light sensor
   int value; // last-read value from sensor
   int threshold; // light/dark sensor value
   int hysterisis; // changes less than this don't count
   unsigned char count_mono; // total number of changes seen (monotonic)
   unsigned char count_120;  // count modulo 120
   unsigned char count_dir;  // encoder count including up/down directions
   unsigned char last_dir; // +1 for counting upward, -1 for counting downward, 0 for no direction
   
   milli_t last_change; // millis() at last changed value
   long period; // milliseconds per change
   boolean high; // if false, low value; if true, high value >threshold

   encoder_t(int pin_, int threshold_)
     :pin(pin_)
   {
     threshold=threshold_;
     hysterisis=threshold_/4;
     count_mono=0;
     count_120=0;
     count_dir=0;
     last_dir=0;
     high=false;
     last_change=milli;
     period=1000;
   }
   
   virtual void change()
   {
   }

   void read(){
      value = analogRead(pin);
      bool changed=false;
      if (high) { // value was high
          if (value<threshold-hysterisis) { changed=true; high=false; }
      } else { // value was low
          if (value>threshold) { changed=true; high=true; }
      }

      if (changed) { // edge detected!
        count_mono++;
        count_120+=last_dir;
        if (count_120>240) count_120=119; // wraparound going negative
        if (count_120>=120) count_120=0; // going positive
        count_dir+=last_dir;
        
        milli_t t=milli;
        period=t-last_change;
        last_change=t;
        change();
      }
    }
};

#endif
