#include "motors.h"
#include "linesensor.h"
#include "encoders.h"
#include "kinematics.h"
#include "pid.h"

// Global definition of the time interval
# define LINE_SENSOR_UPDATE 100
# define MOTOR_UPDATE       2000

// Timestamps for different contexts.
// We will use _ts to show we are using
// this variable as a timestamp.  We will
// record time in a timestamp, so that we
// can work out how much time has elapsed.
unsigned long motor_ts;    
unsigned long line_sensor_ts;

int stop = 1;
LineSensor_c line_sensors; // Line sensor variables/functions
Motors_c motors;           // Motor variables/functions

bool travel_direction;  // to toggle the direction of travel

// put your setup code here, to run once:
void setup() {

  // Initialise the motor gpio
  motors.initialise();
  line_sensors.initialise();

  // Set initial direction
  travel_direction = false;

  // Set initial timestamp values
  motor_ts       = millis();
  line_sensor_ts = millis();

  Serial.begin(9600);
  delay(1000);
  Serial.println("***RESET***");
}


// put your main code here, to run repeatedly:
void loop() {
  // Record the time of this execution
  // of loop for coming calucations
  //  ( _ts = "time-stamp" )
  unsigned long current_ts;
  current_ts = millis();

  // Run our line sensor update
  // every 100ms (10hz).
  // Tracking time for the line sensor (ls)
  elapsed_t = current_ts - line_sensor_ts;
  if( elapsed_t > LINE_SENSOR_UPDATE ) {
    for( int i = 0; i < 5; i++ ) {
      // This line calls a function within your class
      // and stores the returned value.
      float reading = line_sensors.readLineSensor( i );

      Serial.print( reading );
      Serial.print(", ");
    }
    line_sensor_ts = millis();
    Serial.println(""); // to create a new line.

  }

  // Just to test this process:
  // Alternate the motor direction
  // every 2 seconds so that the 3Pi+
  // drives fowards, then backwards.
  elapsed_t = current_ts - motor_ts;
  if( elapsed_t > MOTOR_UPDATE ) {
      // Record when this execution happened
      // for future iterations of loop()
      motor_ts = millis();

      // Toggle motor direction
      if( travel_direction == false ) {
         travel_direction = true;
      } else {
         travel_direction = false;
      }

  }


  // Currently going forwards or backwards?
  // This is checked every loop(), but the state
  // of travel_direction is only changing every
  // MOTOR_UPDATE ms.
  if( travel_direction == true ) {
    motors.setMotorPower( 50, 50 );
  } else {
    motors.setMotorPower( -50, -50 );
  }

  // Delay no longer being used.
  // delay(100)
}
