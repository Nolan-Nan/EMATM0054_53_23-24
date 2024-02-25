// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _KINEMATICS_H
#define _KINEMATICS_H

// Class to track robot position.
class Kinematics_c {
  public:

    float X_I, Y_I =0;
    float theta_I=0;
    const float wheel_radius = 15.5; // Radius of wheels
    const float l = 42.5; // Distance between wheels
    float CPR = 355;
    long previousCountLeft = 0;
    long previousCountRight = 0;
    float mm_per_count = (2 * PI * wheel_radius) / CPR;

  
    // Constructor, must exist.
    Kinematics_c() {

    } 


    // Use this function to update
    // your kinematics
    void update() {
      float X_R, Y_R = 0;
      float theta_R;
      long delta_left = count_left - previousCountLeft;
      long delta_right = count_right - previousCountRight;
      float mean = (delta_left + delta_right)/2;
      
      X_R = mean * mm_per_count;
      theta_R = (delta_left - delta_right) * mm_per_count/(2 * l) ;
      X_I += X_R * cos(theta_I);
      Y_I += X_R * sin(theta_I);
      theta_I += theta_R;
      previousCountLeft = count_left;
      previousCountRight = count_right;
      Serial.print( X_I );
      Serial.print(", ");
      Serial.print( Y_I );
      Serial.print(", ");
      Serial.print( theta_I );
      Serial.println("");
    }

};



#endif
