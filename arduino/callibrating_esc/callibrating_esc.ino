#include <Servo.h>



/* use this program and the serial monitor to calibrate the Traxes XL-5 ESC. 
 *  See https://traxxas.com/support/Programming-Your-Traxxas-Electronic-Speed-Control
 *  Send a 0 to set the throttle to neurtal(90)
 *  send a 1 to set full throttle (180)
 *  send a 2 to set the full reverse(0)
  */
Servo throttle_servo;


void setup() {
  Serial.begin(9600);
  Serial.println("ESC Calibration, Power on, hold button until green and then red. When it flashes once, send a 1, when flashes twice, send a 2");
  throttle_servo.attach(11);  // attaches the servo on pin 9 to the servo object
  
  delay(1);
}

int incomingByte = 0;   // for incoming serial data

void loop() {
        // send data only when you receive data:
        if (Serial.available() > 0) {
                // read the incoming byte:
                incomingByte = Serial.read();

                // say what you got:
                Serial.print("I received: ");
                Serial.println(incomingByte, DEC);

                //Received a zero set neutral
                if(incomingByte == 48){
                  throttle_servo.writeMicroseconds(1000);
                }

                //Received a 1 set full forward
                if(incomingByte == 49){
                  throttle_servo.writeMicroseconds(2000);
                }

                //Received a 2 set full reverse
                if(incomingByte == 50){
                  throttle_servo.writeMicroseconds(1500);
                }
                
        }
}
