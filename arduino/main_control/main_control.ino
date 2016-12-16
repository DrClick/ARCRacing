/*
 * Be sure to first calibrate the ESC. This program controls a Traxxas Slash 2 wheel drive
 * with XL5 speed controller
 */
#include <Servo.h>


const int STEER_IN = 5;
const int STEER_OUT = 9;
const int STEER_BASE = 1475;
const int STEER_MIN = 970;
const int STEER_MAX = 1930;
const int STEER_BIAS = 2; //set this to adjust steering

const int THROTTLE_IN = 6;
const int THROTTLE_OUT = 11;
const int THROTTLE_BASE = 1462;
const int THROTTLE_MIN = 975;
const int THROTTLE_MAX = 1962;

const int LED_GREEN = 2;
const int LED_RED = 3;




//servos
Servo steering_servo;
Servo throttle_servo;

int steer_input;
int throttle_input;

//make sure we have a transmitter so that we can safety stop the car
bool TX_found, TX_high, TX_low;
bool manual_mode = false;



//sets the steering angle between -45, 45 for full left /right respectively. set to zero for straight ahead
void set_steer_angle(int angle){
  angle = angle + STEER_BIAS;
  int map_angle = map(angle, -45, 45, 0, 180);
  steering_servo.write(map_angle);
}

//when using the transmitter, map the steering to the TX output
void set_steer_angle_manual(int sp){  
  int map_angle = map(sp, STEER_MIN, STEER_MAX, -45, 45);
  set_steer_angle(map_angle);
}

//controls the acceleration/deccelration of the robot. -100 is full break to 100 full gas
void set_throttle_position(int sp){
  int map_throttle = map(sp, -100, 100, 1000, 2000);
  throttle_servo.writeMicroseconds(map_throttle);
}

//when using the transmitter, map the throttle to the TX output
void set_throttle_position_manual(int sp){  
  int map_throttle = map(sp, THROTTLE_MIN, THROTTLE_MAX, -100, 100);
  set_throttle_position(map_throttle);
}


//set a reverse speed, 3 second delay to make sure the vehicle is stopped, 
//range is 0 to 100 for min max reverse
void reverse(int sp){
  set_throttle_position(-100);
  delay(3000);
  set_throttle_position(0);
  delay(100);
  int map_throttle = map(sp, 0, 100, -20, -100);
  
  set_throttle_position(map_throttle);
}


//Sweeps the steering servo
void sweep(){
  set_steer_angle(0);
  delay(100);
  //sweep steering
  for(int i=-45;i<45;i++){
    set_steer_angle(i);
    delay(10);
  }
  for(int i=45;i>=-45;i--){
    set_steer_angle(i);
    delay(10);
  }
  set_steer_angle(0);
  delay(100);
}

//This method launches the vehicle off the start line!
void launch(){
  Serial.println("Vector79: launching (wish me luck)...");
  set_throttle_position(0);
  delay(100);
  set_throttle_position(-50);
  delay(100);
  set_throttle_position(100);
  delay(600);
}

void toggle_LED(int led, bool visable){
  if(visable){
    digitalWrite(led, HIGH);
  }
  else{
    digitalWrite(led, LOW);
  }

}

void enter_manual_mode(){
  Serial.println("Vector79: Manual mode triggered from TX");
  manual_mode = true;
  toggle_LED(LED_GREEN, false);
  toggle_LED(LED_RED, true);
}

void setup() {
  Serial.begin(9600);
  Serial.println("Vector79: starting...");

  //set up red/green led indicators
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  toggle_LED(LED_GREEN, false);
  toggle_LED(LED_RED, true);
  
  //read from the controller
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  
  steering_servo.attach(STEER_OUT);  // attaches the servo on pin 9 to the servo object
  throttle_servo.attach(THROTTLE_OUT);  // attaches the servo on pin 9 to the servo object
  
  delay(1);
  sweep();
  Serial.println("Vector79: setup complete in 3s");
  delay(3000);
}



void loop() {

  //loop until the transmitter is toggled full throttle forward and reverse
  while (!TX_found){
      //read in the from the controller
      throttle_input = pulseIn(THROTTLE_IN, HIGH, 25000);
      Serial.println(throttle_input);

      //toggle red led while waiting to arm
      toggle_LED(LED_RED, true);
      delay(10);
      toggle_LED(LED_RED, false);
      delay(10);

      if(throttle_input > THROTTLE_MAX - 100){
        Serial.println("HIGH RX");
        TX_high = true;
      }
      if(throttle_input < THROTTLE_MIN  + 100){
        Serial.println("LOW RX");
        TX_low = true;
      }

      if(TX_low && TX_high){
        TX_found = true;
        toggle_LED(LED_RED, false);
        for(int i=0;i<30;i++){
          toggle_LED(LED_GREEN, true);
          delay(80);
          toggle_LED(LED_GREEN, false);
          delay(80);
        }
        toggle_LED(LED_GREEN, true);

          //WARNING: Uncommenting this will lanuch the robot
        launch();
        //WHOAH stop this bad boy (NOTICE, setting the throttle position to -100 is full breaks, 
        //to go in reverse, use the reverse method
        set_throttle_position(-100);
        delay(3000);
        set_throttle_position(20);
      }
  }//end while init transmitter

  steer_input = pulseIn(STEER_IN, HIGH, 25000);
  throttle_input = pulseIn(THROTTLE_IN, HIGH, 25000);

  //if ever the transmitter is used, go into manual mode
  if(!manual_mode){
    if((abs(steer_input - STEER_BASE) > 100) || (abs(throttle_input - THROTTLE_BASE) > 100)){
      enter_manual_mode();
    }
  }


  if(manual_mode){
    set_steer_angle_manual(steer_input);
    set_throttle_position_manual(throttle_input);
  }
}
