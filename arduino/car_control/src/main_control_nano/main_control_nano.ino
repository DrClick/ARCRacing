/*
   Be sure to first calibrate the ESC. This program controls a Traxxas Slash 2 wheel drive
   with XL5 speed controller
*/
#include <Servo.h>
#include <NewPing.h>
#include <CmdMessenger.h>

#define STEER_IN 5
#define STEER_OUT 9
#define STEER_BASE 1465
#define STEER_MIN 965
#define STEER_MAX 1965
#define STEER_SENSITIVITY 1.2

#define THROTTLE_IN 6
#define THROTTLE_OUT 10
#define THROTTLE_BASE 1465
#define THROTTLE_MIN 965
#define THROTTLE_MAX 1965
#define THROTTLE_SENSITIVITY 3.0

#define LED_GREEN 4
#define LED_RED 3

#define RPM_PIN 2
#define RPM_INTERRUPT 0
#define ROLLING_RPM 400

// left and right sonar trigger at the same time
#define TRIGGER_PIN     7   
#define ECHO_PIN_LEFT   8
#define ECHO_PIN_RIGHT  12

//sonar (centimeters)
#define SONAR_NUM     2 // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.

bool is_logging = true;

/* Define available CmdMessenger commands */
enum {
    cmd_steer,
    cmd_throttle,
    cmd_rpm,
    cmd_sonar,
    cmd_toggle_ebrake,
    cmd_govern_forward,
    cmd_govern_reverse,
    cmd_set_mode,
    cmd_set_steer_bias,
    cmd_info,
    cmd_voltage
};
CmdMessenger commander = CmdMessenger(Serial,',',';','/');

float _STEER_BIAS = 0.0; //set this to adjust steering
int _GOVERNER_F = 20; //cap the forward speed
int _GOVERNER_R = -12;


//servos
Servo steering_servo;
Servo throttle_servo;

float steer_input = STEER_BASE;
float throttle_input = THROTTLE_BASE;

float throttle_pos;
float steering_angle;


//make sure we have a transmitter so that we can safety stop the car
bool TX_found, TX_high, TX_low;
bool manual_mode = false;

//rpm monitoring
volatile byte half_revolutions;
unsigned int rpm;
unsigned long timeold;
int rpm_decay;


//emergency breaking
bool emergency_breaking = false;

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//sets the steering angle between -45, 45 for full left /right respectively. set to zero for straight ahead
void set_steer_angle(float angle) {
  steering_angle = angle;
  commander.sendBinCmd(cmd_steer,steering_angle);

  
  //apply bias which accounts for physical issues, not model issues
  //we dont want our intented steer angle (0 for straight) to
  //reflect any issue in physical bias of the car
  angle = angle + _STEER_BIAS;

  int map_angle_ms = int(map(angle, -45, 45, 1000, 2000));
  steering_servo.write(map_angle_ms);
}

//when using the transmitter, map the steering to the TX output
void set_steer_angle_manual(float sp) {
  float map_angle = mapfloat(sp, STEER_MIN, STEER_MAX, -45.0, 45.0);
  set_steer_angle(map_angle);
}

//controls the acceleration/deccelration of the robot. -100 is full break to 100 full gas

void set_throttle_position(float pos) {
  //dont allow input past the limits
  pos = min(_GOVERNER_F, pos);

  throttle_pos = pos;
  commander.sendBinCmd(cmd_throttle, throttle_pos);

  //note!!!!! the 1000 and 2000 here are the min and max PWM that the controller accepts
  int map_throttle = int(map(pos, -100, 100, 1000, 2000));
  

  throttle_servo.writeMicroseconds(map_throttle);
}

//when using the transmitter, map the throttle to the TX output
void set_throttle_position_manual(float sp) {
  float map_throttle = mapfloat(sp, THROTTLE_MIN, THROTTLE_MAX, -100, 100);
  set_throttle_position(map_throttle);
}


//set a reverse speed, 3 second delay to make sure the vehicle is stopped,
//range is 0 to 100 for min max reverse
void reverse(int sp) {
  set_throttle_position(-100);
  delay(3000);
  set_throttle_position(0);
  delay(100);
  int map_throttle = map(sp, 0, 100, -20, -100);

  set_throttle_position(map_throttle);
}


//Sweeps the steering servo
void sweep() {
  set_steer_angle(0);
  delay(100);
  //sweep steering
  for (int i = -45; i < 45; i++) {
    set_steer_angle(i);
    delay(10);
  }
  for (int i = 45; i >= -45; i--) {
    set_steer_angle(i);
    delay(10);
  }
  set_steer_angle(0);
  delay(100);
}

void toggle_LED(int led, bool visable) {
  if (visable) {
    digitalWrite(led, HIGH);
  }
  else {
    digitalWrite(led, LOW);
  }
}

void enter_manual_mode() {
  commander.sendCmd(cmd_info, "Manual mode triggered from TX");
  manual_mode = true;
  toggle_LED(LED_GREEN, false);
  toggle_LED(LED_RED, true);
}

void enter_auto_mode() {
  commander.sendCmd(cmd_info, "Entering Auto mode");
  manual_mode = false;
  toggle_LED(LED_GREEN, true);
  toggle_LED(LED_RED, false);
  steer_input = STEER_BASE;
  throttle_input = THROTTLE_BASE;

}

void enter_arming_mode(){
  commander.sendCmd(cmd_info, "Please arm transmitter by toggeleing full left and right!!!");
  TX_found = false;
  TX_high = false;
  TX_low = false;

  while (!TX_found) {
    //read in the from the controller
    steer_input = pulseIn(STEER_IN, HIGH, 25000);
    throttle_input = pulseIn(THROTTLE_IN, HIGH, 25000);
    //Serial.println(steer_input);
    //Serial.println(throttle_input);

    //toggle red led while waiting to arm
    toggle_LED(LED_RED, true);
    delay(10);
    toggle_LED(LED_RED, false);
    delay(10);

    if (steer_input > (STEER_MAX - 100) && !TX_high) {
      commander.sendCmd(cmd_info, "LEFT RX");
      TX_high = true;
    }
    if (steer_input < (STEER_MIN  + 100) && !TX_low) {
      commander.sendCmd(cmd_info, "RIGHT RX");
      TX_low = true;
    }

    if (TX_low && TX_high) {
      TX_found = true;
      toggle_LED(LED_RED, false);
      for (int i = 0; i < 30; i++) {
        toggle_LED(LED_GREEN, true);
        delay(80);
        toggle_LED(LED_GREEN, false);
        delay(80);
      }
      enter_auto_mode();
    }
    
  }//end while init transmitter
}

void monitor_rpm(){
  half_revolutions++;
}

void setup() {
  Serial.begin(115200);
  commander.sendCmd(cmd_info, "starting...");
  attachInterrupt(RPM_INTERRUPT, monitor_rpm, RISING);
  digitalWrite(RPM_PIN, HIGH);

  //set up red/green led indicators
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  toggle_LED(LED_GREEN, false);
  toggle_LED(LED_RED, true);

  //read from the controller
  pinMode(STEER_IN, INPUT);
  pinMode(THROTTLE_IN, INPUT);

  steering_servo.attach(STEER_OUT);  // attaches the servo on pin 9 to the servo object
  throttle_servo.attach(THROTTLE_OUT);  // attaches the servo on pin 9 to the servo object

  delay(1);
  sweep();
  commander.sendCmd(cmd_info, "setup complete in 1s");
  delay(1000);

  //init rpm
  half_revolutions = 0;
  rpm = 0;
  timeold = 0;


  attach_commander_callbacks(); 

  enter_arming_mode();
}



void loop() {

  //look for commands
  commander.feedinSerialData();

  //buffer the inputs, from the controller, they are noisy 
//  steer_input = (steer_input * 2 + pulseIn(STEER_IN, HIGH, 250000)) / 3.0;
//  throttle_input = (throttle_input * 2 + pulseIn(THROTTLE_IN, HIGH, 250000)) / 3.0;

  steer_input = pulseIn(STEER_IN, HIGH, 250000);
  throttle_input = pulseIn(THROTTLE_IN, HIGH, 250000);

  //if ever the transmitter is used, go into manual mode
  if (!manual_mode) {
    if ((abs(steer_input - STEER_BASE) > 250) || (abs(throttle_input - THROTTLE_BASE) > 250)) {
      enter_manual_mode();
    }
  }
  else {
    set_steer_angle_manual(steer_input);
    set_throttle_position_manual(throttle_input);
  }

  //output rpms
  write_rpms();

  //decay rpms incase vehicle has stopped
  rpm_decay++;
  if (rpm_decay >20){
    rpm = 0;
    rpm_decay = 0;
    commander.sendBinCmd(cmd_rpm, rpm);
   }
 

  
 
}


//commander callbacks
void on_cmd_steer(void){
    if(!manual_mode) set_steer_angle(commander.readBinArg<float>());
}

void on_cmd_throttle(void){
    if(!manual_mode) set_throttle_position(commander.readBinArg<float>());
}

void on_cmd_toggle_ebrake(void){
  emergency_breaking = commander.readBinArg<bool>();
}

void on_cmd_govern_forward(void){
  int max_throttle = commander.readBinArg<int>();
  if(max_throttle > 0 && max_throttle <= 100) _GOVERNER_F = max_throttle;
}

void on_cmd_govern_reverse(void){
  int max_throttle = commander.readBinArg<int>();
  if(max_throttle < 0 && max_throttle >= -100) _GOVERNER_R = max_throttle;
}

void on_cmd_set_mode(void){
  if (commander.readBinArg<bool>()) enter_auto_mode();
  else enter_manual_mode();
}

void on_cmd_set_steer_bias(void){
  _STEER_BIAS = commander.readBinArg<int>();
}

/* Attach callbacks for CmdMessenger commands */
void attach_commander_callbacks(void) {
    commander.attach(cmd_steer, on_cmd_steer);
    commander.attach(cmd_throttle, on_cmd_throttle);
    commander.attach(cmd_toggle_ebrake, on_cmd_toggle_ebrake);
    commander.attach(cmd_govern_forward, on_cmd_govern_forward);
    commander.attach(cmd_govern_reverse, on_cmd_govern_reverse);
    commander.attach(cmd_set_mode, on_cmd_set_mode);
    commander.attach(cmd_set_steer_bias, on_cmd_set_steer_bias);
}

void write_rpms(){
  if (half_revolutions >= 2) {
     rpm = 30 * 1000/(millis() - timeold) * half_revolutions;
     rpm_decay = 0;
     timeold = millis();
     half_revolutions = 0;
     commander.sendBinCmd(cmd_rpm, rpm);
  }
}
