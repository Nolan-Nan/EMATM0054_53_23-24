#include "motors.h"
#include "linesensor.h"

# define active_threshold 1000
# define BiasPWM 20
# define MaxTurnPWM 40

Motors_c motors;
LineSensor_c line_sensors; // Line sensor variables/functions
bool start = true;

void setup() {
  motors.initialise();
  line_sensors.initialise();

  Serial.begin(9600);
  delay(1000);
  Serial.println("***RESET***");
}

void loop() {
  float W = weightedMeasurement();
  if (start){
    if (!lineDetected(2) && !lineDetected(1) && !lineDetected(3)&& !lineDetected(0)&& !lineDetected(4)){
      motors.setMotorPower( 15,15 );
      delay(200);
    }else{
      motors.setMotorPower( 0,0 );
      delay(200);
      start=false;
    }
  }else{
    /*
    // Greater than or equal to
    if(lineDetected(1)||lineDetected(0) && !lineDetected(3)) {
      motors.setMotorPower( 13, 20 );
      delay(200);
    // Greater than or equal to
    } else if( lineDetected(3) || lineDetected(4) && !lineDetected(1)) {
      motors.setMotorPower( 20, 13 );
      delay(200);
    } else if( !lineDetected(2) && !lineDetected(1) && !lineDetected(3)&& !lineDetected(0)&& !lineDetected(4)) {
      motors.setMotorPower( 0,0 );
      delay(200);
    }else {
      motors.setMotorPower( 15, 15 );
      delay(200);
    }*/
    if(lineDetected(1) || lineDetected(3) || lineDetected(2)) {
      int LeftPWM = BiasPWM - (W * MaxTurnPWM);
      int RightPWM = BiasPWM + (W * MaxTurnPWM);
      Serial.print( "," );
      Serial.print( LeftPWM );
      Serial.print( "," );
      Serial.print( RightPWM );
      Serial.print( "\n");
      motors.setMotorPower( LeftPWM, RightPWM );
    }else{
      motors.setMotorPower( 0, 0);
      Serial.print( "\n");
    }
  }

}

bool lineDetected(int number){
  bool detected = false;
  int sensor = line_sensors.readLineSensor( number );
  if (sensor >= active_threshold){
    detected = true;
  }
  return detected;
}


float weightedMeasurement(){
  float R1= line_sensors.readLineSensor( 1 );
  float R2= line_sensors.readLineSensor( 2 );
  float R3= line_sensors.readLineSensor( 3 );
  Serial.print(R1,4);
  Serial.print( "," );
  Serial.print(R2,4);
  Serial.print( "," );
  Serial.print( R3,4 );
  Serial.print( "," );
  //sum DN2 & DN4
  float sum = R1+R3;
  //normalise reading
  float N1 = R1/sum;
  float N3 = R3/sum;
  Serial.print(N1,4);
  Serial.print( "," );
  Serial.print( N3,4 );
  Serial.print( "," );
  //double weighting
  N1 = N1 *2;
  N3 = N3 *2;
  Serial.print(N1,4);
  Serial.print( "," );
  Serial.print( N3,4 );
  Serial.print( "," );
  float weight = N1-N3;
  Serial.print( weight,4 );
  return weight;
}

void lineFollowing(){
  if(lineDetected(1)||lineDetected(2) || lineDetected(3)) {
    int W = weightedMeasurement();
    int LeftPWM = BiasPWM + (W * MaxTurnPWM);
    int RightPWM = BiasPWM - (W * MaxTurnPWM);
    motors.setMotorPower( LeftPWM, RightPWM );
  }else{
    motors.setMotorPower( 0, 0);
  }
}


