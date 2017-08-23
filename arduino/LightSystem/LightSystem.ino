/*
 Fade

 This example shows how to fade an LED on pin 9
 using the analogWrite() function.

 The analogWrite() function uses PWM, so if
 you want to change the pin you're using, be
 sure to use another PWM capable pin. On most
 Arduino, the PWM pins are identified with 
 a "~" sign, like ~3, ~5, ~6, ~9, ~10 and ~11.

 This example code is in the public domain.
 */

#define HEAD_LED 11
#define LEFT_LED 10
#define RIGHT_LED 9
#define TAIL_LED 6
#define BRAKE_LED 5

#define RED_LED 4
#define GREEN_LED 7
#define BLUE_LED 8

#define running_brightness 100
#define low_beam 150
#define high_beam 255
#define TURN_SIGNAL_FADE 10   // how many points to fade the LED by


int brightness = 150;   // how bright the LED is
int turn_signal_fade = TURN_SIGNAL_FADE;
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete


bool bright_on = false;
bool running_on = false;
bool brake_on = false;
bool left_on = true;
bool right_on = false;
bool status_on = false;
bool red_on = false;
bool blue_on = false;
bool green_on = false;


// the setup routine runs once when you press reset:
void setup() {
  pinMode(LEFT_LED, OUTPUT);
  pinMode(RIGHT_LED, OUTPUT);
  pinMode(BRAKE_LED, OUTPUT);
  pinMode(TAIL_LED, OUTPUT);
  pinMode(HEAD_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  Serial.begin(115200);
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);
}

// the loop routine runs over and over again forever:
void loop() {
  unsigned long currentMillis = millis();

  if (stringComplete) {
    Serial.println(inputString);
    inputString.trim();
    
    if(inputString == "BRAKE_ON")     {brake_on = true;}
    if(inputString == "BRAKE_OFF")    {brake_on = false;}
    if(inputString == "LEFT_ON")      {left_on = true;}
    if(inputString == "LEFT_OFF")     {left_on = false;}
    if(inputString == "RIGHT_ON")     {right_on = true;}
    if(inputString == "RIGHT_OFF")    {right_on = false;}
    if(inputString == "BRIGHT_ON")    {bright_on = true;}
    if(inputString == "BRIGHT_OFF")   {bright_on = false;}
    if(inputString == "RUNNING_ON")   {running_on = true;}
    if(inputString == "RUNNING_OFF")  {running_on = false;}
    if(inputString == "STATUS_ON")    {status_on = true;}
    if(inputString == "STATUS_OFF")   {status_on = false;}
    if(inputString == "RED_ON")       {red_on = true;}
    if(inputString == "RED_OFF")      {red_on = false;}
    if(inputString == "BLUE_ON")      {blue_on = true;}
    if(inputString == "BLUE_OFF")     {blue_on = false;}
    if(inputString == "GREEN_ON")     {green_on = true;}
    if(inputString == "GREEN_OFF")    {green_on = false;}


    // clear the string:
    inputString = "";
    stringComplete = false;
  }

  
  if(left_on){blink(LEFT_LED, currentMillis);}
  else if(!running_on){analogWrite(LEFT_LED, 0);}
  if(right_on){blink(RIGHT_LED, currentMillis);}
  else if(!running_on){analogWrite(RIGHT_LED, 0);}
 


  //switch lights as indicated
   if(running_on){
     analogWrite(TAIL_LED, 50);
     if(!bright_on){analogWrite(HEAD_LED, 50);}
     if(!left_on){analogWrite(LEFT_LED, 50);}
     if(!right_on){analogWrite(RIGHT_LED, 50);}
     analogWrite(BRAKE_LED, 50);
  }
  else{
    analogWrite(TAIL_LED, 0);
    analogWrite(HEAD_LED, 0);
  }
  if(brake_on){analogWrite(BRAKE_LED, 255);}
  else if(!running_on){analogWrite(BRAKE_LED, 0);}
  
  if(bright_on){analogWrite(HEAD_LED, 255);}

  if(status_on){
    if(red_on){digitalWrite(RED_LED, HIGH);}
    else{digitalWrite(RED_LED, LOW);}
    
    if(blue_on){digitalWrite(BLUE_LED, HIGH);}
    else{digitalWrite(BLUE_LED, LOW);}
    if(green_on){digitalWrite(GREEN_LED, HIGH);}
    else{digitalWrite(GREEN_LED, LOW);}
  }
  else{
    digitalWrite(RED_LED, LOW);
    digitalWrite(BLUE_LED, LOW);
    digitalWrite(GREEN_LED, LOW);
  }

  
  
}

unsigned long previousMillisTurn = 0; 
void blink(int led, unsigned long m){
  
  if (m - previousMillisTurn >= 30){
    previousMillisTurn = m;
    brightness = brightness + turn_signal_fade;
    if (brightness <= 150 || brightness >= 255) {
      turn_signal_fade = -turn_signal_fade;
    }
    analogWrite(led, brightness);
  }
}


/*
 * COMMANDS ------
 * HLEFT
 * HRIGHT
 * SLEFT
 * SRIGHT
 * BRAKE
 * RUNNING
 * BRIGHTS
 */

 


void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}
