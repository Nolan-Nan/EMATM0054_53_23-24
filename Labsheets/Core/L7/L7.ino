#include "motors.h"
#include "linesensor.h"
#include "encoders.h"
#include "kinematics.h"
#include "pid.h"


# define active_threshold 1000
# define BiasPWM 20
# define MaxTurnPWM 40
# define BUZZER_PIN 6

// Declare your different possible states here by enumerating them.
# define STATE_INITIAL  0
# define STATE_DRIVE_FORWARDS   1
# define STATE_FOUND_LINE  2
# define STATE_FOLLOW_LINE 3
# define STATE_REDISCOVER 4
# define STATE_ROTATE 5
# define STATE_BACK 6

// This will hold which state your robot is in. It will receive
// the numbers set above, but your code will be readable because
// strings used in the #define's
int state;
float R0,R1,R2,R3,R4,ls_read[5];
Motors_c motors;
LineSensor_c line_sensors; // Line sensor variables/functions
Kinematics_c kinematic;
PID_c left_pid;
PID_c right_pid;
PID_c heading_pid;

bool isInitial = false; // is ready tomoce forward
bool foundLine = false; // find the line to follow
bool followLine = false;
bool rediscover = false;
bool readyToBack = false;
bool inbox = true; // if the robot cross the start box set to true

// Global variables to track encoder counts
long prev_count_left;
long prev_count_right;

// Global variable to track time per iteration of loop()
unsigned long prev_time = 0;

// Global variable to store estimation of rotation velocity
float rotation_velocity_left;
float rotation_velocity_right;
const float demand = 450; 

void setup() {
  // Setup encoders when the robot is
  // powered on.
  motors.initialise();
  setupEncoder0();
  setupEncoder1();
  left_pid.initialize(0.1,0.05,0);
  right_pid.initialize(0.1,0.05,0);
  
  Serial.begin(9600);
  delay(1000);
  Serial.println("***RESET***");
}

void loop() {
  calculateRotationVelocity();
  /*
  // Calculate feedback signal using P control
  float feedback_signal_left = pid.update(demand, rotation_velocity_left);
  */
  float feedback_signal_left = left_pid.update(demand, rotation_velocity_left);
  float feedback_signal_right = right_pid.update(demand, rotation_velocity_right);
  Serial.print(demand);


  Serial.print(",");
  Serial.print(feedback_signal_left);
  Serial.print(",");
  Serial.println(feedback_signal_right);
  motors.setMotorPower(feedback_signal_left,feedback_signal_right);

  //motors.setMotorPower(feedback_signal_left,feedback_signal_right);
  // Store previous encoder counts for next iteration
  // Delay for stability (adjust as needed)
  delay(20); // 20ms interval as mentioned
}

void calculateRotationVelocity() {
  // Capture a change of encoder counts
  long delta_left = count_left - prev_count_left;
  long delta_right = count_right - prev_count_right;
  
  // Capture the time elapsed between loop iterations
  unsigned long current_time = millis();
  unsigned long time_elapsed = current_time - prev_time;
  
  // Update previous time for next iteration
  prev_time = current_time;
  
  // Calculate rotation velocity for left wheel
  rotation_velocity_left = (float)abs(delta_left) / time_elapsed * 1000; // Convert to counts per second
  
  // Calculate rotation velocity for right wheel
  rotation_velocity_right = (float)abs(delta_right) / time_elapsed * 1000; // Convert to counts per second
  
  // Store previous encoder counts for next iteration
  prev_count_left = count_left;
  prev_count_right = count_right;
  Serial.print(rotation_velocity_left);
  Serial.print(",");
  Serial.print(rotation_velocity_right);
  Serial.print(",");
}


