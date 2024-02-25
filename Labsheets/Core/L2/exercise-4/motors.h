// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _MOTORS_H
#define _MOTORS_H
#define FWD LOW
#define REV HIGH

// Class to operate the motor(s).
class Motors_c {
  public:

    int L_PWM_PIN;
    int L_DIR_PIN;
    int R_PWM_PIN;
    int R_DIR_PIN;

    // Constructor, must exist.
    Motors_c() {

    } 

    // Use this function to 
    // initialise the pins and 
    // state of your motor(s).
    void initialise() {
      L_PWM_PIN = 10;
      L_DIR_PIN = 16;
      R_PWM_PIN = 9;
      R_DIR_PIN = 15;

      // Set all the motor pins as outputs.
      pinMode(L_PWM_PIN, OUTPUT);
      pinMode(L_DIR_PIN, OUTPUT);
      pinMode(R_PWM_PIN, OUTPUT);
      pinMode(R_DIR_PIN, OUTPUT);

      // Set initial direction (HIGH/LOW)
      // for the direction pins.
      digitalWrite(L_DIR_PIN, HIGH);  // Set initial direction for left motor
      digitalWrite(R_DIR_PIN, LOW);   // Set initial direction for right motor

      // Set initial power values for the PWM Pins.
      analogWrite(L_PWM_PIN, 0);  // Initially, keep motors turned off
      analogWrite(R_PWM_PIN, 0);
    }

    
    void setMotorPower( float left_pwm, float right_pwm ) {
      Serial.println(left_pwm);
      Serial.println(right_pwm);

      if (left_pwm<0){
        digitalWrite(L_DIR_PIN, REV);
      }else{
        digitalWrite(L_DIR_PIN, FWD);
      }
      if (right_pwm<0){
        digitalWrite(R_DIR_PIN, REV);
      }else{
        digitalWrite(R_DIR_PIN, FWD);
      }

      if (abs(left_pwm)<=50){
        analogWrite( L_PWM_PIN, abs(left_pwm));
      }else{
        analogWrite( L_PWM_PIN, 0);
        Serial.println("the pwm set is not in range [-50,50]");
      }
      if (abs(right_pwm)<=50){
        analogWrite( R_PWM_PIN, abs(right_pwm));
      }else{
        analogWrite( R_PWM_PIN, 0);
        Serial.println("the pwm set is not in range [-50,50]");
      }
    
  }


    
};



#endif
