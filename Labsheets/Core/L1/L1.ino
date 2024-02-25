
// Including the standard math library to
// utilise sin and cos functions, and the
// TWO_PI constant.
# include <math.h>

int toggle_duration = 1;
float a;
float b;
float c;

void flash_leds( int delay_ms ) {

  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay( delay_ms );                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay( delay_ms );

}

void beep( int toggle_duration ) {
  digitalWrite(6, HIGH);   
  delayMicroseconds(toggle_duration);                       
  digitalWrite(6, LOW);    
  delayMicroseconds(toggle_duration);
}

// the setup function runs once when you press reset or power the board
void setup() {

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(6, OUTPUT);

  //Start a serial connection
  Serial.begin(9600);

  // Wait for stable connection, report reset.
  delay(1000);
  Serial.println("***RESET***");

  a = TWO_PI;
  b = a;
  c = b;

}

// the loop function runs over and over again forever
void loop() {

  flash_leds( 500 );                   // flash LEDS
  beep(toggle_duration);
  a += 0.01;
  b = sin( a * 20 );
  c = cos( a * 10 );
  //Serial.print( toggle_duration );
  Serial.print( ",");
  Serial.print( a );
  Serial.print( ",");
  Serial.print( b );
  Serial.print( ",");
  Serial.print( c );

  Serial.print( "\n" );   // Finish with a newline
  //Serial.println( "" ); // either of these two work.

  delay(50);
  toggle_duration += 100;
}
