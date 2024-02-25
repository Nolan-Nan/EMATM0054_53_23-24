#include "motors.h"
#include "linesensor.h"
#include "encoders.h"
#include "kinematics.h"
#include "pid.h"

#define LED_PIN 13  // Pin to activate the orange LED
boolean led_state;  // Variable to "remember" the state
                    // of the LED, and toggle it.
int stop = 1;
Motors_c motors;
// put your setup code here, to run once:
void setup() {

  // Initialise the motor gpio
  motors.initialise();

  Serial.begin(9600);
  delay(1000);
  Serial.println("***RESET***");

  // Set LED pin as an output
  pinMode( LED_PIN, OUTPUT );

  // Set initial state of the LED
  led_state = false;
}


// put your main code here, to run repeatedly:
void loop() {

  // Using an if statement to toggle a variable
  // with each call of loop()
  if( led_state == true ) {
    led_state = false;
  } else {
    led_state = true;
  }

  // We use the variable to set the
  // debug led on or off on the 3Pi+
  digitalWrite( LED_PIN, led_state );
  // test to set motors
  if (stop<60){
    motors.setMotorPower(stop,stop);
    delay(1000);
    stop += 1;
  }else if(stop<120){
    motors.setMotorPower(60-stop,60-stop);
    delay(1000);
    stop += 1;
  }else{
    // Example to stop the robot indefinitely.
    // Only a hard reset will break the loop.
      motors.setMotorPower(0,0);
      Serial.println("Program Halted");
      delay(500);
    }
  Serial.println("loop");
  delay(500);
}
