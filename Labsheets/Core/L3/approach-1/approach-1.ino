// The pin used to activate the IR LEDs
# define EMIT_PIN    11       // Documentation says 11.
# define LS_LEFT_PIN 12     // Complete for DN1 pin
# define LS_MIDLEFT_PIN 18  // Complete for DN2 pin
# define LS_MIDDLE_PIN 20  // Complete for DN3 pin
# define LS_MIDRIGHT_PIN 21 // Complete for DN4 pin
# define LS_RIGHT_PIN 22    // Complete for DN5 pin

int max1,max2,max3=0;
int min1,min2,min3=1000;
// Store our pin numbers into an array.  We can then
// use this as a "find and replace" for the pin number
// using just an index of the array from 0 to 4 (1-5)
int ls_pins[5] = {LS_LEFT_PIN,
                  LS_MIDLEFT_PIN,
                  LS_MIDDLE_PIN,
                  LS_MIDRIGHT_PIN,
                  LS_RIGHT_PIN };
// Global "permanent" store of line sensor values
int ls_value[5];

// Example of the function readLineSensor() to
// update just 1 specific sensor reading at a time.
//
// Input:  int which_sensor, used to identify
//         which of the 5 lines sensors is
//         being read.
// Output: Updates global variables.
void readLineSensor( int which_sensor ) {
  ls_value[ which_sensor ] = analogRead( ls_pins[ which_sensor ] );
}

// Example of the function readAllLineSensors() to
// update all line sensors in 1 function call.
//
// Input:  None
//
// Output: Updates global variables.
void readAllLineSensors() {

  // Loop across all sensors.
  for( int i = 0; i < 5; i++ ) {

    // Save current value to global variable.
    ls_value[i] = analogRead( ls_pins[i] );
  }
}

// Example to configure the line sensors.  
// Input:  none.
// Output: uses pinMode() to set the correct
//         state of the line sensor pins.
void setupLineSensors() {

  // Loop across each pin, setting pinMode().
  for( int i = 0; i < 5; i++ ) {
    pinMode(ls_pins[i], INPUT_PULLUP);
  }
}

void setup() {

   // configure the central line sensor pin
   // as input.
  /*pinMode(A0, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);*/
  setupLineSensors();
   // Configure the EMIT pin as output and
   // high.  This will active some infra-red
   // (IR) LEDs for our sensors to read.
   pinMode( EMIT_PIN, OUTPUT );
   digitalWrite( EMIT_PIN, HIGH );

   // Configure the Serial port
   Serial.begin(9600);

}

void loop() {

  // Capture time we start this operation
  unsigned long start_time = micros();
  unsigned long start_mil = millis();
   // A variable to save a reading
   int dn2;
   int dn3;
   int dn4;

   // Use ADC, assign result to variable
  //dn2 = analogRead(A0);
  //dn3 = analogRead(A2);
  //dn4 = analogRead(A3);
  /*
  max1 = max(max1,dn2);
  min1 = min(min1,dn2);
  max2 = max(max2,dn3);
  min2 = min(min2,dn3);
  max3 = max(max3,dn4);
  min3 = min(min3,dn4);
  
   // Report back to computer over Serial
   Serial.print( max1);
   Serial.print( "," );
   Serial.print( min1);
   Serial.print( "," );
   Serial.print( max2);
   Serial.print( "," );
   Serial.print( min2);
   Serial.print( "," );
   Serial.print( max3);
   Serial.print( "," );
   Serial.print( min3);
   Serial.print( "\n" );

   Serial.print( dn2);
   Serial.print( "," );
   Serial.print( dn3);
   Serial.print( "," );
   Serial.print( dn4);
   Serial.print( "\n" );
   delay(100);
  */
  readAllLineSensors();
   // Capture the time after the line sensors read
  unsigned long end_time = micros();
  unsigned long elapsed_time = end_time - start_time;

  Serial.print(ls_value[0]);
   Serial.print( "," );
   Serial.print( ls_value[1]);
   Serial.print( "," );
   Serial.print(ls_value[2]);
   Serial.print( "," );
   Serial.print( ls_value[3]);
   Serial.print( "," );
   Serial.print(ls_value[4]);
   Serial.print( "\n" );
  Serial.println( elapsed_time );
}
