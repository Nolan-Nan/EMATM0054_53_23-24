# define EMIT_PIN    11    // Documentation says 11.
# define LS_LEFT_PIN 12     // Complete for DN1 pin
# define LS_MIDLEFT_PIN 18  // Complete for DN2 pin
# define LS_MIDDLE_PIN 20  // Complete for DN3 pin
# define LS_MIDRIGHT_PIN 21 // Complete for DN4 pin
# define LS_RIGHT_PIN 22    // Complete for DN5 pin

// Store our pin numbers into an array, which means
// we can conveniently select one later.
// ls(line sensor)_pin
int ls_pins[5] = {LS_LEFT_PIN,
                  LS_MIDLEFT_PIN,
                  LS_MIDDLE_PIN,
                  LS_MIDRIGHT_PIN,
                  LS_RIGHT_PIN };

# define TIMEOUT_THRESHOLD 3000

# define MAX_SAMPLES 10

float results[ MAX_SAMPLES ]; // An array of MAX_SAMPLES length

void setup() {

  // Set some initial pin modes and states
  pinMode( EMIT_PIN, INPUT ); // Set EMIT as an input (off)
  pinMode( LS_LEFT_PIN, INPUT );     // Set line sensor pin to input

  // Start Serial, wait to connect, print a debug message.
  Serial.begin(9600);
  delay(1500);
  Serial.println("***RESET***");

} // End of setup()


void loop() {


  // Complete the steps referring to the pseudocode block
  // Algorithm 1.
  // The first steps have been done for you.
  for (int i=0;i<MAX_SAMPLES;i++){
    
    // This line is complete, but it needs to be
    // within a for loop construct.  Complete this.
    results[i] = readLineSensor( 42 ); 
    delay(200);
  }

  // The section below has been completed for you.
  // Use this section to build a similar construct
  // around the code above, which was Algorithm 1.

  // The robot will be "stuck" here forever, because
  // the condition true will never change.
  // Therefore, the robot will Serial Print the
  // results back to your computer every 1000ms.
  while( true ) {
      Serial.println("Results: ");
      for( int i = 0; i < MAX_SAMPLES; i++ ) {
        Serial.println( results[i] );
      }
      delay(1000);
  }



} // End of loop()

// A function to read a line sensor.
// Specify which sensor to read with number.
// Number should be a value between 0 and 4
float readLineSensor( int number ) {

    // These two if statements should be
    // completed to prevent a memory error.
    // What would be a good value to return that
    // would not be mistaken for a sensor reading?
    if( number < 0 ) {
        return -1;
    }
    if( number > 4 ) {
        return -1;
    }

    pinMode( EMIT_PIN, OUTPUT );
    digitalWrite( EMIT_PIN, HIGH );


    pinMode( ls_pins[ number ], OUTPUT );
    digitalWrite( ls_pins[ number ], HIGH );
    delayMicroseconds( 10 );

    pinMode( ls_pins[ number ], INPUT );
    unsigned long start_time = micros();

    while( digitalRead( ls_pins[ number ]) == HIGH ) {
      // Check for time-out condition
      if (micros() - start_time > TIMEOUT_THRESHOLD) {
          // Handle time-out condition
          Serial.println("Sensor reading time-out");
          return -1; // Return a special value to indicate time-out
      }
    }

    unsigned long end_time = micros();

    pinMode( EMIT_PIN, INPUT );

    unsigned long elapsed_time = end_time - start_time;

    return (float)elapsed_time;
}

