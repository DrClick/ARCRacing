/*
 * Be sure to first calibrate the ESC
 */
#include <Servo.h>

Servo steering_servo;
Servo throttle_servo;


void set_steer_angle(int angle){
  int map_angle = map(angle, -90, 90, 0, 180);
  steering_servo.write(map_angle);
}

void set_throttle_position(int sp){
  //Add 1 to the position so that zero is actually 1. This seems to keep the ESC alive.
  if(sp==0}{
    sp = 1;
  }
  
  int map_throttle = map(sp, -100, 100, 0, 180);
  throttle_servo.write(map_throttle);
}

void sweep(){
  set_steer_angle(0);
  delay(100);
  //sweep steering
  for(int i=-90;i<90;i++){
    set_steer_angle(i);
    delay(10);
  }
  for(int i=90;i>=-90;i--){
    set_steer_angle(i);
    delay(10);
  }
  set_steer_angle(0);
  delay(100);
}

void launch(){
  set_throttle_position(0);
  delay(100);

  //head backwards for 400ms
  for (int i=0;i>=-80;i--){
    set_throttle_position(i);
    delay(5);
  }

  //nail the gas
  for (int i=-80;i<100;i+10){
    set_throttle_position(i);
    delay(2);
  }
}


void setup() {
  steering_servo.attach(9);  // attaches the servo on pin 9 to the servo object
  throttle_servo.attach(11);  // attaches the servo on pin 9 to the servo object
  
  delay(1);
  sweep();
  delay(2000);
  launch();
  delay(1000);

  //WHOAH stop this bad boy
  set_throttle_position(0);
  delay(2000);
}


void loop() {
  set_throttle_position(20);
  set_steer_angle(90);
  delay(10000);
  set_throttle_position(0);
  delay(200);
  set_throttle_position(-30);
  delay(10000);
  set_throttle_position(0);
  delay(1000);
  sweep();
  
}
