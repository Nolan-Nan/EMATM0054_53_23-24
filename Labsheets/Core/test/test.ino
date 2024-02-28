#include "motors.h"
#include "linesensor.h"
#include "encoders.h"
#include "kinematics.h"
#include "pid.h"


# define active_threshold 1000
# define BiasPWM 20
# define MaxTurnPWM 40
# define BUZZER_PIN 6


float R0,R1,R2,R3,R4;
Motors_c motors;
LineSensor_c line_sensors; // Line sensor variables/functions
Kinematics_c kinematic;
PID_c left_pid;
PID_c right_pid;
PID_c heading_pid;

long prev_count_left;
long prev_count_right;
// Global variable to track time per iteration of loop()
unsigned long prev_time = 0;
// Global variable to store estimation of rotation velocity
float rotation_velocity_left;
float rotation_velocity_right;
const float demand = 300; 

void setup() {
  // Insert necessary initialising here.  e.g classes, pin modes
  // and states, Serial.begin()
  motors.initialise();
  line_sensors.initialise();
  pinMode(BUZZER_PIN, OUTPUT);
  heading_pid.initialize(0.01, 0.01, 0.001);

  // Setup encoders when the robot is
  // powered on.
  setupEncoder0();
  setupEncoder1();
  
  Serial.begin(9600);
  delay(1000);
  Serial.println("***RESET***");
}

void loop() {
  updateSystem();
  kinematic.update();
  lineFollowing();
}

bool lineDetected(int number){
  bool detected = false;
  int sensor = line_sensors.readLineSensor( number );
  if (sensor >= active_threshold){
    detected = true;
  }
  return detected;
}

void updateSystem(){
  R0= line_sensors.readLineSensor( 0 );
  R1= line_sensors.readLineSensor( 1 );
  R2= line_sensors.readLineSensor( 2 );
  R3= line_sensors.readLineSensor( 3 );
  R4= line_sensors.readLineSensor( 4 );
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
   Serial.println(W);
  // Check if any of the sensors on the line
  if(lineDetected(1) || lineDetected(3) || lineDetected(2)) {
    float feedback = heading_pid.update(0, W);
    int LeftPWM = BiasPWM + (feedback * MaxTurnPWM);
    int RightPWM = BiasPWM - (feedback * MaxTurnPWM);
    Serial.println(feedback);
    motors.setMotorPower( LeftPWM, RightPWM );
  }else {
    motors.setMotorPower( 0, 0);

  }
}


