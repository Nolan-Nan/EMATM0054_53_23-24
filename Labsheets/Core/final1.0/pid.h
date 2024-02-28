#include "Arduino.h"
// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _PID_H
#define _PID_H

// Class to contain generic PID algorithm.
class PID_c {
  public:

    float K_p; // Proportional gain
    float K_i; // Integral gain
    float K_d; // derivative gain
    float e;  // Error signal
    float e_sum;
    float prev_e; // Previous error
    unsigned long prev_time; // Previous time for integral term calculation
    float p_term; // Proportional term
    float i_term;
    float d_term;
    
    // Constructor, must exist.
    PID_c() {

    } 

    void initialize(float init_Kp, float init_Ki, float init_Kd) {
      K_p = init_Kp;
      K_i = init_Ki;
      K_d = init_Kd;
      e = 0.0;
      e_sum = 0.0;
      prev_time = millis();
      prev_e = 0;
      p_term = 0.0;
      i_term = 0.0;
      d_term = 0.0;
    }

    float update(float demand, float measurement) {
      // Current time
      unsigned long current_time = millis();
      // Time elapsed since last update
      unsigned long dt = current_time - prev_time;

      // Error calculation
      e = demand - measurement;
      // Proportional term calculation
      p_term = K_p * e;

      e_sum += e * dt / 1000.0; // Trapezoidal integration
      i_term = K_i * e_sum;
      d_term = K_d * ( e - prev_e) * 1000 / dt;
      prev_time = current_time;

      // Return feedback signal
      return p_term + i_term + d_term;
    }

};



#endif
