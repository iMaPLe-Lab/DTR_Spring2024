//Rename this to main.cpp or <something>.ino if you're in Arduino
//Starts the ESC, sets it to be bidirectionnal, then beeps, then makes it go one way then the other over and over at low speeds

#include "WiFi.h"
#include "Arduino.h"
#include "Wire.h"
#include "Adafruit_BNO08x.h"
#include "wifi_setup.h"
#include "sensors.h"
#include "pid.h"
#include <Arduino.h>
#include "DShotESC.h"
#include <ESP32Servo.h>

Servo myservo;  // create servo object to control a servo
// 16 servo objects can be created on the ESP32
int servoPin = 9;
#define PEAKSPEED 500
#define SINE_DURATION 10000.f //duration of the full cycle, in millis
int pos = 0;  

void setReports(sh2_SensorId_t reportType, long report_interval);
void sensorSetup();

// Initialize data variables
int16_t tfDist = 0;       // Distance to object in centimeters
int16_t tfFlux = 0;       // Signal strength or quality of return signal
int16_t tfTemp = 0;       // Internal temperature of Lidar sensor chip

//////// Define global variables ////////////
int leftJoystickX   = 0;  // (-511 - 512) left X axis
int leftJoystickY   = 0;  // (-511 - 512) left Y axis
int leftThrottle    = 0; // for altitude increase (0-1023)
int rightThrottle   = 0;
int Brakebutton     = 0; // for altitude decrease (0-1023)
int rightJoystickX  = 0; // (-511 - 512) right X axis
int rightJoystickY  = 0; // (-511 - 512) right Y axis
int catchs          = 0;
int attKill         = 0;
int R2              = 0;
bool XButton = 0;
bool toggle = 0;
bool last_toggle = 0;
bool catch_flag = 0;

int* ptrR2 = &R2;
int* ptrLeftJoystickX   = &leftJoystickX;
int* ptrLeftJoystickY   = &leftJoystickY;
int* ptrLeftThrottle    = &leftThrottle;
int* ptrRightThrottle   = &rightThrottle;
int* ptrBrakebutton     = &Brakebutton;
int* ptrRightJoystickX  = &rightJoystickX;
int* ptrRightJoystickY  = &rightJoystickY;
int* ptrCatchs          = &catchs;
int* ptrAttKill         = &attKill;

bool* ptrXButton = &XButton;
unsigned long lastUpdate = 0; // Stores the last update time
const unsigned long updateInterval = 3000; // Update interval in milliseconds (500ms)


double target_alt = 100; // mm
int powerLeft = 0;
int powerRight = 0;
int powerUp = 0;
int forward = 0;
int turn = 0;
int up = 0;
/// @esc ///
DShotESC esc0;
DShotESC esc1;
DShotESC esc2;

float roll;
float pitch;
float yaw;

static int speedEscAltitude = 0, speedEscLeft = 0, speedEscRight = 0;
int* ptr_speedEscAltitude = &speedEscAltitude;
int* ptr_speedEscLeft = &speedEscLeft;
int* ptr_speedEscRight = &speedEscRight;

bool is_turning = false;

void escUpdateTask(void * parameter) {
   for (;;) {
    if(ptr_speedEscAltitude != nullptr) esc0.sendThrottle3D(*ptr_speedEscAltitude);
    if(ptr_speedEscLeft != nullptr) esc1.sendThrottle3D(*ptr_speedEscLeft);
    if(ptr_speedEscRight != nullptr) esc2.sendThrottle3D(*ptr_speedEscRight);
    vTaskDelay(pdMS_TO_TICKS(100)); // Adjusts for 10Hz update rate
  }
}


void motorSetup()
{
	Serial.begin(115200);

  // altitude motor
  pinMode(GPIO_NUM_35, OUTPUT);
	esc0.install(GPIO_NUM_35, RMT_CHANNEL_0);
	esc0.init();
  esc0.throttleArm(1700);
	esc0.setReversed(true);
	esc0.set3DMode(true);

  // Left motor
  pinMode(GPIO_NUM_37, OUTPUT);
	esc1.install(GPIO_NUM_37, RMT_CHANNEL_1);
	esc1.init();
  esc1.throttleArm(1700);
	esc1.setReversed(true);
	esc1.set3DMode(true);

  // Right motor.
  pinMode(GPIO_NUM_18, OUTPUT);
	esc2.install(GPIO_NUM_18, RMT_CHANNEL_2);
	esc2.init();
  esc2.throttleArm(1700);
	esc2.setReversed(true);
	esc2.set3DMode(true);

  // Create the ESC update task
  xTaskCreate(escUpdateTask, "ESC Update Task", 2048, NULL, 1, NULL);

}

void setup() {
  Serial.begin(115200);

  motorSetup();
  sensorSetup();

  myservo.attach(servoPin, 1000, 2000); // attaches the servo on pin 18 to the servo object

  //////////// Wifi setup ////////////
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.println(WiFi.localIP());
  // Start the server
  server.begin();

  
  //do not touch this, I don't know how it works 
  esc0.throttleArm(1700);
  esc1.throttleArm(1700);
  esc2.throttleArm(1700);
}


void loop() {

  WiFiClient client = server.available();
  if (client) {
    Serial.println("Client connected");
    while (client.connected()) {
      if (client.available()) {
        Data data = readControllerData(client); // get controller values

        processData(data, ptrLeftJoystickX, ptrLeftJoystickY, ptrLeftThrottle, ptrRightJoystickY, ptrRightThrottle, ptrXButton); // map recieved controller values to pointers
        // Serial.print("\n Up and down: ");
        // Serial.print(leftJoystickY);
        // Serial.print(", ");
        // Serial.print(leftJoystickX);
        // Serial.print(", Thrust: ");
        // Serial.print(rightJoystickX);
        // Serial.print(", Left&Right: ");
        // Serial.print(rightJoystickY);
        // Serial.print("R2:");
        // Serial.print(R2);
        // Serial.print("\n");
        
        /******************* ALITITUDE CONTROL *******************/
        *ptr_speedEscAltitude = leftJoystickY*6; // up and down 
        

        /******************* DIRECTION/YAW CONTROL *******************/
        readIMU();
        Serial.print("Yaw: ");
        Serial.print(ypr.yaw);
        Serial.print(", Target Yaw: ");
        Serial.println(target_yaw);

        int leftMotorSpeed = map(max(0, -rightJoystickY), 0, 100, 0, 100);
        int rightMotorSpeed = map(max(0, rightJoystickY), 0, 100, 0, 100);

        if (is_turning && abs(rightJoystickY) < 5) { // Just stopped turning
          is_turning = false;
          target_yaw = ypr.yaw; // set yaw position to hold
        }
        Serial.print("Yaw PID value: ");
        double yaw_pid = constrain(computePID_yaw(target_yaw, ypr.yaw), -350, 350);
        double yaw_pid_fw = constrain(computePID_yaw_fw(target_yaw, ypr.yaw), -200, 200);
        Serial.println(yaw_pid);

        if(rightJoystickY!=0){ // if turning
          is_turning = true;

          if(leftThrottle>0){ // turn and move forward
            if(rightJoystickY>0){
              *ptr_speedEscRight = leftThrottle*(1.0-float(rightMotorSpeed)/100.0);
              *ptr_speedEscLeft = -leftThrottle;
            } else if(rightJoystickY<0){
              *ptr_speedEscRight = leftThrottle;
              *ptr_speedEscLeft = -leftThrottle*(1.0-float(leftMotorSpeed)/100.0);
            }
          } else if(leftThrottle==0){ // turn in place
            if(rightJoystickY<0){
              *ptr_speedEscRight = 500*(float(leftMotorSpeed)/100.0);
              *ptr_speedEscLeft = 500*(float(leftMotorSpeed)/100.0);
            } else if(rightJoystickY>0){
              *ptr_speedEscRight = -500*(float(rightMotorSpeed)/100.0);
              *ptr_speedEscLeft = -500*(float(rightMotorSpeed)/100.0);
            }
          }

        } else if(rightThrottle == 0 ){ // backward?
          *ptr_speedEscLeft =  constrain(leftThrottle*7, -700, 700);
          *ptr_speedEscRight = -constrain(leftThrottle*7, -700, 700);
        } else if(rightThrottle != 0){ // forward?
          
          *ptr_speedEscLeft = -constrain(rightThrottle*7, -700, 700);
          *ptr_speedEscRight =  constrain(rightThrottle*7,-700, 700);
        } 
        // else if(R2==0 && rightJoystickX == 0 ){ // hold yaw angle
        //   // *ptr_speedEsc1 = -yaw_pid;
        //   // *ptr_speedEsc2 = -yaw_pid;
        // }


        /******************* CAPTURE CONTROL *******************/
        if(last_toggle == 1 && XButton ==0 ){
          catch_flag = !catch_flag;
        }
              
        last_toggle = XButton;

        unsigned long currentMillis = millis(); // Current time in milliseconds
        unsigned long lastCheckTime = 0; // Timestamp of the last check
        if (catch_flag && currentMillis - lastCheckTime >= 150) { // If x is true, move servo to 90 degrees
          myservo.write(90);
          // Serial.println("Servo moved to 90 degrees");
          lastCheckTime = currentMillis; // Update last check time
          toggle = false; // Reset x back to false after acting on it
        } else { // If x is false, keep or move servo to 0 degrees
          myservo.write(0);
        }

      
      }
    }
    client.stop();
    Serial.println("Client disconnected");
  }
}