#include "motors.h"
#include "linesensor.h"
#include "encoders.h"
#include "kinematics.h"


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

bool isInitial = false; // is ready tomoce forward
bool foundLine = false; // find the line to follow
bool followLine = false;
bool rediscover = false;
bool readyToBack = false;
bool inbox = true; // if the robot cross the start box set to true

// put your setup code here, to run once:
void setup() {

  // Insert necessary initialising here.  e.g classes, pin modes
  // and states, Serial.begin()
  motors.initialise();
  line_sensors.initialise();
  pinMode(BUZZER_PIN, OUTPUT);

  // Setup encoders when the robot is
  // powered on.
  setupEncoder0();
  setupEncoder1();
  
  Serial.begin(9600);
  delay(1000);
  Serial.println("***RESET***");

  // Set system state to intial  before robot begins to operate.
  state = STATE_INITIAL;
}

// put your main code here, to run repeatedly:
void loop() {
  

  updateSystem();
  // flow control is managed in updateState()
  updateState();

  kinematic.update();
  /*
  Serial.print(state);
  Serial.print(", ");*/
  // This is the basic structure for a FSM  Based on the value
  // of "state" variable, run appropriate code for robot behaviour.
  if( state == STATE_INITIAL ) {
    calibration();     
  } else if( state == STATE_DRIVE_FORWARDS ) {
    driveForwards();     
  } else if( state == STATE_FOUND_LINE ) {
    foundLineBeeps();
  } else if( state == STATE_FOLLOW_LINE ) {
    lineFollowing();
  } else if( state == STATE_REDISCOVER ){
    rediscoverBeep();
  } else if(state == STATE_ROTATE){
    rotateToOrigin();
  } else if(state == STATE_BACK){
    goBack();
  }else {

          // catch situations where the robot attempts to move into an unknown state.
          Serial.print("System Error, Unknown state: ");
          Serial.println( state );

          // These functions might be useful to write.
          stopRobot();
  }
}

void updateSystem(){
  R0= line_sensors.readLineSensor( 0 );
  R1= line_sensors.readLineSensor( 1 );
  R2= line_sensors.readLineSensor( 2 );
  R3= line_sensors.readLineSensor( 3 );
  R4= line_sensors.readLineSensor( 4 );
  ls_read[0] = R0;
  ls_read[1] = R1;
  ls_read[2] = R2;
  ls_read[3] = R3;
  ls_read[4] = R4;
}

// A function to update the system variables, and to
// cause transitions between system states.
void updateState() {
  switch (state) {
    case STATE_INITIAL:
      // Check if the robot is ready to the drive forwards state
      if (isInitial) {
        state = STATE_DRIVE_FORWARDS;
      }
      break;
    case STATE_DRIVE_FORWARDS:
      // Check if the line is detected
      if (foundLine) {
        state = STATE_FOUND_LINE;
      }
      break;
    case STATE_FOUND_LINE:
      // Check if the line is detected
      if (followLine) {
        state = STATE_FOLLOW_LINE;
      }
      break;
    case STATE_FOLLOW_LINE:
      // Check if the line is detected
      if (rediscover) {
        if(millis()>40000){
          state = STATE_ROTATE;
          stopRobot();
          delay(1000);
        }else{ 
          state = STATE_REDISCOVER;
        }
      }
      break;
    case STATE_REDISCOVER:
      if (followLine) {
        state = STATE_FOLLOW_LINE;
      }
      break;
    case STATE_ROTATE:
      // Check if the line is detected
      if (readyToBack) {
        state = STATE_BACK;
      }
      break;
  }

}


float weightedMeasurement(){
  
  //sum DN2 & DN4
  float sum = R1+R3;
  //normalise reading
  float N1 = R1/sum;
  float N3 = R3/sum;
  //double weighting
  N1 = N1 *2;
  N3 = N3 *2;
  float weight = N1-N3;
  return weight;
}


void lineFollowing() {
  float W = weightedMeasurement();
  if (lineDetected(4)){
    motors.setMotorPower(BiasPWM, -BiasPWM);
    delay(100);
  }
  // Check if any of the sensors on the line
  if(lineDetected(1) || lineDetected(3) || lineDetected(2)) {
    int LeftPWM = BiasPWM - (W * MaxTurnPWM);
    int RightPWM = BiasPWM + (W * MaxTurnPWM);
    motors.setMotorPower( LeftPWM, RightPWM );
  }else {
    rediscover = true;
    followLine = false;
    motors.setMotorPower( 0, 0);
  }
}



bool lineDetected(int number){
  bool detected = false;
  if (ls_read[number] >= active_threshold){
    detected = true;
  }
  return detected;
}

void rediscoverBeep(){
  if (lineDetected(2)){
    digitalWrite(BUZZER_PIN, HIGH); 
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    followLine = true;
    rediscover = false;
    stopRobot();
  }else{
    motors.setMotorPower(-BiasPWM, BiasPWM);
    delay(100);
  }
}


// write this function to have your
// robot beep 5 times, across a total
// of 5 seconds.
void initializingBeeps() {

  // Loop to generate 5 beeps
  for (int i = 0; i < 2; i++) {
    // Generate the beep tone
    digitalWrite(BUZZER_PIN, HIGH); 
    delay(100);
    digitalWrite(BUZZER_PIN, LOW); 
    delay(900);
  }
  digitalWrite(BUZZER_PIN, LOW); 
  delay(100);
}


void calibration() {
  initializingBeeps();
  isInitial=true;
}


// Write code that will command your
// robot to drive forwards straight
// until it detects the course line.
void driveForwards() {
  motors.setMotorPower(21, 20);
  delay(200);
  if (lineDetected(0) || lineDetected(1) || lineDetected(2) || lineDetected(3) || lineDetected(4)){
    if(inbox){
      delay(500);
      inbox = false;
    }else{
      foundLine = true;
    }
  }
  if (foundLine){
    stopRobot();
  }
}


// Write code that will deactivate all
// motors, and beep once for one second,
// then do nothing.
void foundLineBeeps() {
  digitalWrite(BUZZER_PIN, HIGH); 
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);
  if (lineDetected(2)){
    followLine = true;
    stopRobot();
  }else{
    motors.setMotorPower(-BiasPWM, BiasPWM);
    delay(100);
  }
}

void rotateToOrigin() {
  float angle_to_origin = atan2(kinematic.Y_I, kinematic.X_I) - kinematic.theta_I;

  while (angle_to_origin > PI) {
    angle_to_origin -= 2 * PI;
  }
  while (angle_to_origin < -PI) {
      angle_to_origin += 2 * PI;
  }

  float angle_threshold = 0.05;

  if (abs(angle_to_origin) < angle_threshold) {
    stopRobot();
    readyToBack = true;
  } else {
    if (angle_to_origin > 0) {
      motors.setMotorPower(-BiasPWM, BiasPWM);
    } else {
      motors.setMotorPower(BiasPWM, -BiasPWM);
    }
  }

  delay(10); 
}

void goBack(){
  float distance = sqrt(pow(kinematic.X_I, 2) + pow(kinematic.Y_I, 2));
  if (distance > 25){
    motors.setMotorPower(21.5, BiasPWM);
    delay(100);
  }else{
    stopRobot();
  }
}


// Function to stop the robot
void stopRobot() {
  motors.setMotorPower(0, 0 );
  delay(100);
}


