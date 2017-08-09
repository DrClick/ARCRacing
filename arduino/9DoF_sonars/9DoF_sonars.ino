/*see https://github.com/mikedotalmond/arduino-pulseInWithoutDelay
 * and https://gist.github.com/mikedotalmond/6044960
 * for inspiration for ultrasonic sensors
 * This sketch controls the sonars and the 9Degree of Freedom Sensor and reports
 * them on the serial bus for listenging to by ROS
 * 
 */

#include <PulseInZero.h>
#include <PulseInOne.h>

// The SFE_LSM9DS1 library requires both Wire and SPI be
// included BEFORE including the 9DS1 library.
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

//create IMU
LSM9DS1 imu;

#define LSM9DS1_M	0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG	0x6B // Would be 0x6A if SDO_AG is LOW
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION 13.5 // Declination (degrees) in SF, Ca 8-1-2017

//sonar
#define TRIGGER_LEFT_PIN 5
#define TRIGGER_RIGHT_PIN 4

const float SpeedOfSound       = 343.2; // ~speed of sound (m/s) in air, at 20°C         
const float MicrosecondsPerMillimetre   = 1000.0 / SpeedOfSound; // microseconds per millimetre - sound travels 1 mm in ~2.9us
const float MicrosecondsToMillimetres  = (1.0 / MicrosecondsPerMillimetre);
const float MicrosecondsToMillimetres2 = MicrosecondsToMillimetres / 2.0; // beam travels the distance twice... so halve the time.
unsigned long lastTime = 0;
int pingTimer = 0;
int pingDelay = 16; // milliseconds between ping pulses
bool useLeft = true; //alternate between left and right sonar.

void setup() 
{
  Serial.begin(115200);

  //config IMU
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    while (1)
      ;
  }

  //sonar
  pinMode(TRIGGER_LEFT_PIN, OUTPUT);
  pinMode(TRIGGER_RIGHT_PIN, OUTPUT);
  digitalWrite(TRIGGER_LEFT_PIN, LOW);
  digitalWrite(TRIGGER_RIGHT_PIN, LOW);
  
  PulseInOne::setup(leftComplete);
  PulseInZero::setup(rightComplete);
 
}

void loop()
{
  unsigned long t = millis();
  unsigned long dt = t - lastTime;
  lastTime = t;
  pingTimer += dt;
  
  if(pingTimer > pingDelay){
    checkIMU();
    pingTimer = 0;
    if (useLeft){
      pingLeft();
      useLeft = false;
    }
    else{
      pingRight();
      useLeft = true;
    }
  }
}

void checkIMU(){
  // Update the sensor values whenever new data is available
  if ( imu.gyroAvailable() ){imu.readGyro();}
  if ( imu.accelAvailable() ){imu.readAccel();}
  if ( imu.magAvailable() ){imu.readMag();}
  
  printGyro();  // Print "G: gx, gy, gz"
  printAccel(); // Print "A: ax, ay, az"
  printMag();   // Print "M: mx, my, mz"
  printAttitude(imu.ax, imu.ay, imu.az, -imu.my, -imu.mx, imu.mz);
}

void printGyro()
{  
  Serial.println("G:" + 
    String(imu.calcGyro(imu.gx), 2) + "," + 
    String(imu.calcGyro(imu.gy), 2) + "," + 
    String(imu.calcGyro(imu.gz), 2));
}


void printAccel()
{  
  Serial.println("A:" + 
    String(imu.calcAccel(imu.ax), 2) + "," + 
    String(imu.calcAccel(imu.ay), 2) + "," + 
    String(imu.calcAccel(imu.az), 2));
}

void printMag()
{  
  Serial.println("M:" + 
    String(imu.calcMag(imu.mx), 2) + "," + 
    String(imu.calcMag(imu.my), 2) + "," + 
    String(imu.calcMag(imu.mz), 2));
}

// Calculate pitch, roll, and heading.
// Pitch/roll calculations take from this app note:
// http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf?fpsp=1
// Heading calculations taken from this app note:
// http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
void printAttitude(float ax, float ay, float az, float mx, float my, float mz)
{
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));
  
  float heading;
  if (my == 0)
    heading = (mx < 0) ? PI : 0;
  else
    heading = atan2(mx, my);
    
  heading -= DECLINATION * PI / 180;
  
  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  else if (heading < 0) heading += 2 * PI;
  
  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;
  
  Serial.println("X:" + String(pitch,2) + "," + String(roll,2) + "," + String(heading,2));
}

void pingLeft(){
  digitalWrite(TRIGGER_LEFT_PIN, HIGH);
  digitalWrite(TRIGGER_LEFT_PIN, LOW);
  
  // start listening out for the echo pulse on interrupt 0
  PulseInZero::begin();
}

void pingRight(){
  digitalWrite(TRIGGER_RIGHT_PIN, HIGH);
  digitalWrite(TRIGGER_RIGHT_PIN, LOW);
  
  // start listening out for the echo pulse on interrupt 0
  PulseInOne::begin();
}

void leftComplete(unsigned long duration){
  pingComplete(duration, 'L');
}
void rightComplete(unsigned long duration){
  pingComplete(duration, 'R');
}
void pingComplete(unsigned long duration, char sensor) {

  unsigned int mm = MicrosecondsToMillimetres2 * duration;
  
  if(mm > 4000){
       mm = 4000;
  } else {
      Serial.print(sensor);
      Serial.print(':');
      Serial.println(mm);
  }
}
