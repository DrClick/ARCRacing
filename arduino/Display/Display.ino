/*
  LiquidCrystal Library - display() and noDisplay()

 Demonstrates the use a 16x2 LCD display.  The LiquidCrystal
 library works with all LCD displays that are compatible with the
 Hitachi HD44780 driver. There are many of them out there, and you
 can usually tell them by the 16-pin interface.

 This sketch prints "Hello World!" to the LCD and uses the
 display() and noDisplay() functions to turn on and off
 the display.

 The circuit:
 * LCD RS pin to digital pin 12
 * LCD Enable pin to digital pin 11
 * LCD D4 pin to digital pin 5
 * LCD D5 pin to digital pin 4
 * LCD D6 pin to digital pin 3
 * LCD D7 pin to digital pin 2
 * LCD R/W pin to ground
 * 10K resistor:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 3)

 Library originally added 18 Apr 2008
 by David A. Mellis
 library modified 5 Jul 2009
 by Limor Fried (http://www.ladyada.net)
 example added 9 Jul 2009
 by Tom Igoe
 modified 22 Nov 2010
 by Tom Igoe

 This example code is in the public domain.

 http://www.arduino.cc/en/Tutorial/LiquidCrystalDisplay

 */

// include the library code:
#include <LiquidCrystal.h>

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);

//init listening for serial commands
String inputString = "";         // a String to hold incoming data
boolean stringComplete = false;  // whether the string is complete


void setup() {
  Serial.begin(115200);
  inputString.reserve(200);
  
  // set up the LCD's number of columns and rows:
  lcd.begin(20, 4);
  // Print a message to the LCD.
  lcd.setCursor(0,1);
  lcd.write("      VECTOR-79     ");
  lcd.setCursor(0,2);
  lcd.write("Autonomous RC Racing");
  delay(3000);
  
  lcd.setCursor(0,0);
  lcd.write("System is starting  ");
  lcd.setCursor(0,1);
  lcd.write("IP:                 ");
  lcd.setCursor(0,2);
  lcd.write("S:        T:        ");
  lcd.setCursor(0,3);
  lcd.write("IN:   V  TEMP:    C ");
}

void loop() {
  if (stringComplete) {
    //message are in the form K - Message
    //where K is
    //P - Temp
    //I - IP
    //S - Steering
    //T - Throttle
    //V - Voltage
    //1 - line 1
    //2 - line 2

    inputString.trim();
    String message = inputString.substring(4);
    char first = inputString.charAt(0);

    Serial.println(first);
    Serial.println(message);
    switch (first) {
      case 'P':
        lcd.setCursor(15,3);
        break;
      case 'V':
        lcd.setCursor(4,3);
        break;
      case 'I':
        lcd.setCursor(0, 1);
        message = "IP: " + message;
        break;
      case 'S':
        lcd.setCursor(3, 2);
        break;
      case 'T':
        lcd.setCursor(13, 2);
        break;
      case '1':
        lcd.setCursor(0,0);
        message = message + "                    ";
        message = message.substring(0,20);
        break;
      case '2':
        lcd.setCursor(0,1);
        message = message + "                    ";
        message = message.substring(0,20);
        break;
        
      default: 
        // if nothing else matches, do the default
        // default is optional
      break;
    }
    lcd.print(message);

    // clear the string:
    inputString = "";
    stringComplete = false;
  }
}


void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

