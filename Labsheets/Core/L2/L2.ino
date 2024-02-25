# define L_PWM_PIN 10
# define L_DIR_PIN 16
# define R_PWM_PIN 9
# define R_DIR_PIN 15

#define FWD LOW
#define REV HIGH

int stop = 1;
// Runs once.
void setup() {

  // Set all the motor pins as outputs.
  // There are 4 pins in total to set.
  // ...
  pinMode(L_PWM_PIN, OUTPUT);
  pinMode(L_DIR_PIN, OUTPUT);
  pinMode(R_PWM_PIN, OUTPUT);
  pinMode(R_DIR_PIN, OUTPUT);

  // Set initial direction (HIGH/LOW)
  // for the direction pins.
  digitalWrite(L_DIR_PIN, HIGH);  // Set initial direction for left motor
  digitalWrite(R_DIR_PIN, LOW);   // Set initial direction for right motor

  // Set initial power values for the PWM
  // Pins.
  analogWrite(L_PWM_PIN, 0);  // Initially, keep motors turned off
  analogWrite(R_PWM_PIN, 0);


  // Start serial, send debug text.
  Serial.begin(9600);
  delay(1000);
  Serial.println("***RESET***");

}

// Repeats.
void loop() {
  if (stop<60){
    setMotorPower(stop,stop);
    delay(1000);
    stop += 1;
  }else if(stop<120){
    setMotorPower(60-stop,60-stop);
    delay(1000);
    stop += 1;
  }else{
    // Example to stop the robot indefinitely.
    // Only a hard reset will break the loop.
      analogWrite( L_PWM_PIN, 0);
      analogWrite( R_PWM_PIN, 0);
      Serial.println("Program Halted");
      delay(500);
    }

    // An empty loop can block further uploads.
    // A small delay to prevent this for now.
    delay(5);
}

/*
 * Sets the power of the motors using analogWrite().
 * This function sets direction and PWM (power).
 * This function catches all errors of input PWM.
 *  inputs:
 *     pwm   accepts negative, 0 and positve
 *           values.  Sign of value used to set
 *           the direction of the motor.  Values
 *           are limited in range [ ??? : ??? ].
 *           Magnitude used to set analogWrite().
 */
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