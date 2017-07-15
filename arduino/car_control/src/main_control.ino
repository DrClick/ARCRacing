/*
   Be sure to first calibrate the ESC. This program controls a Traxxas Slash 2 wheel drive
   with XL5 speed controller
*/
#include <Servo.h>


const int STEER_IN = 5;
const int STEER_OUT = 9;
const int STEER_BASE = 1277;
const int STEER_MIN = 847;
const int STEER_MAX = 1710;
const float STEER_SENSITIVITY = 1.0;

const int THROTTLE_IN = 6;
const int THROTTLE_OUT = 11;
const int THROTTLE_BASE = 1277;
const int THROTTLE_MIN = 847;
const int THROTTLE_MAX = 1710;
const float THROTTLE_SENSITIVITY = 1.0;

const int LED_GREEN = 2;
const int LED_RED = 3;

int _STEER_BIAS = 0; //set this to adjust steering
int _GOVERNER = 1600; //cap the forward speed


//servos
Servo steering_servo;
Servo throttle_servo;

float steer_input = STEER_BASE;
float throttle_input = THROTTLE_BASE;

int throttle_pos;
int steering_angle;

// Input commands
String command = "";// a string to hold incoming data
boolean command_complete = false;  // whether the string is complete


//make sure we have a transmitter so that we can safety stop the car
bool TX_found, TX_high, TX_low;
bool manual_mode = false;



//sets the steering angle between -45, 45 for full left /right respectively. set to zero for straight ahead
void set_steer_angle(int angle) {
  //dont set unless a delta has been acheived
  if (abs(steering_angle - angle) / 2.0 < STEER_SENSITIVITY){
    return;
  }
  
  steering_angle = angle;
  
  //apply bias which accounts for physical issues, not model issues
  //we dont want our intented steer angle (0 for straight) to
  //reflect any issue in physical bias of the car
  angle = angle + _STEER_BIAS;

  int map_angle = map(angle, -45, 45, 0, 180);
  steering_servo.write(map_angle);
}

//when using the transmitter, map the steering to the TX output
void set_steer_angle_manual(float sp) {
  int map_angle = map(int(sp), STEER_MIN, STEER_MAX, -45, 45);
  set_steer_angle(map_angle);
}

//controls the acceleration/deccelration of the robot. -100 is full break to 100 full gas
void set_throttle_position(int pos) {
  //dont set unless a delta has been acheived
  if (abs(throttle_pos - pos)/2.0 < THROTTLE_SENSITIVITY){
    return;
  }
  
  throttle_pos = pos;

  int map_throttle = map(pos, -100, 100, 1000, 2000);
  //set minimum is 1400 for reverse and 1550 for slow forward
  if (map_throttle < 1200) {
    map_throttle = 1200;
  }
  if (map_throttle > _GOVERNER) {
    map_throttle = _GOVERNER;
  }

  throttle_servo.writeMicroseconds(map_throttle);
}

//when using the transmitter, map the throttle to the TX output
void set_throttle_position_manual(float sp) {
  int map_throttle = map(int(sp), THROTTLE_MIN, THROTTLE_MAX, -100, 100);
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
  Serial.println("Vetor79: Manual mode triggered from TX");
  manual_mode = true;
  toggle_LED(LED_GREEN, false);
  toggle_LED(LED_RED, true);
}

void enter_auto_mode() {
  Serial.println("Vetor79: Entering Auto mode");
  manual_mode = false;
  toggle_LED(LED_GREEN, true);
  toggle_LED(LED_RED, false);
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
  Serial.println("Vector79: setup complete in 1s");
  delay(1000);

  // reserve 200 bytes for the inputString:
  command.reserve(200);
}



void loop() {

  //loop until the transmitter is toggled full throttle forward and reverse
  while (!TX_found) {
    //read in the from the controller
    throttle_input = pulseIn(THROTTLE_IN, HIGH, 25000);

    //toggle red led while waiting to arm
    toggle_LED(LED_RED, true);
    delay(10);
    toggle_LED(LED_RED, false);
    delay(10);

    if (throttle_input > (THROTTLE_MAX - 100)) {
      Serial.println("Vector79: HIGH RX");
      TX_high = true;
    }
    if (throttle_input < (THROTTLE_MIN  + 100)) {
      Serial.println("Vector79: LOW RX");
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

  //buffer the steer input, its noisy
  steer_input = (steer_input * 9 + pulseIn(STEER_IN, HIGH, 25000)) / 10.0;

  throttle_input = (throttle_input * 4 + pulseIn(THROTTLE_IN, HIGH, 25000)) / 5.0;

  //if ever the transmitter is used, go into manual mode
  if (!manual_mode) {
    if ((abs(steer_input - STEER_BASE) > 100) || (abs(throttle_input - THROTTLE_BASE) > 100)) {
      enter_manual_mode();
    }
  }


  if (manual_mode) {
    set_steer_angle_manual(steer_input);
    set_throttle_position_manual(throttle_input);
  }

  //look for command
  if (command_complete) {
    //get out of manual mode
    if (manual_mode && command == "AUTO") {
      enter_auto_mode();

    }

    if (!manual_mode) {
      //throttle
      if (command.startsWith("T")) {
        command.replace("T", " ");
        command.trim();
        set_throttle_position(command.toInt());
      }

      //steering
      if (command.startsWith("S")) {
        command.replace("S", " ");
        command.trim();
        set_steer_angle(command.toInt());
      }

      //steering bias
      if (command.startsWith("B")) {
        command.replace("B", " ");
        command.trim();
        _STEER_BIAS = command.toInt();
      }

      //governer
      if (command.startsWith("G")) {
        command.replace("G", " ");
        command.trim();
        if (command.toInt() > THROTTLE_BASE) {
          _GOVERNER = command.toInt();
        }
      }

      //manual mode
      if (command.startsWith("M")) {
        enter_manual_mode();
      }


    }

    Serial.println("RCV:" + command);
    // clear the string:
    command = "";
    command_complete = false;
  }


  //log the current state
  Serial.println(String("V79 ") + throttle_pos + ":" + steering_angle);
}

/*
  SerialEvent occurs whenever a new data comes in the
  hardware serial RX.  This routine is run between each
  time loop() runs, so using delay inside loop can delay
  response.  Multiple bytes of data may be available.
*/
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    command += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      command.trim();
      command_complete = true;
    }
  }
}
