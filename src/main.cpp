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
int Throttlebutton  = 0; // for altitude increase (0-1023)
int Brakebutton     = 0; // for altitude decrease (0-1023)
int rightJoystickX  = 0; // (-511 - 512) right X axis
int rightJoystickY  = 0; // (-511 - 512) right Y axis
int catchs          = 0;
int attKill         = 0;
int R2              = 0;
bool X = 0;
bool toggle = 0;
bool last_toggle = 0;
bool catch_flag = 0;

int* ptrR2 = &R2;
int* ptrLeftJoystickX   = &leftJoystickX;
int* ptrLeftJoystickY   = &leftJoystickY;
int* ptrThrottlebutton  = &Throttlebutton;
int* ptrBrakebutton     = &Brakebutton;
int* ptrRightJoystickX  = &rightJoystickX;
int* ptrRightJoystickY  = &rightJoystickY;
int* ptrCatchs          = &catchs;
int* ptrAttKill         = &attKill;

bool* ptrX = &X;
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

static int speedEsc0 = 0, speedEsc1 = 0, speedEsc2 = 0;
int* ptr_speedEsc0 = &speedEsc0;
int* ptr_speedEsc1 = &speedEsc1;
int* ptr_speedEsc2 = &speedEsc2;

bool is_turning = false;

void escUpdateTask(void * parameter) {
   for (;;) {
    if(ptr_speedEsc0 != nullptr) esc0.sendThrottle3D(*ptr_speedEsc0);
    if(ptr_speedEsc1 != nullptr) esc1.sendThrottle3D(*ptr_speedEsc1);
    if(ptr_speedEsc2 != nullptr) esc2.sendThrottle3D(*ptr_speedEsc2);
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

        processData(data, ptrLeftJoystickX, ptrLeftJoystickY, ptrRightJoystickX, ptrRightJoystickY, ptrR2, ptrX); // map recieved controller values to pointers
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
        *ptr_speedEsc0 =leftJoystickY*6; // up and down 
        

        /******************* DIRECTION/YAW CONTROL *******************/
        readIMU();
        Serial.print("Yaw: ");
        Serial.print(ypr.yaw);
        Serial.print(", Target Yaw: ");
        Serial.println(target_yaw);
        // Serial.print(", Roll: ");
        // Serial.print(ypr.roll);
        // Serial.print(", Pitch: ");
        // Serial.println(ypr.pitch);

        int leftMotorSpeed = map(max(0, -rightJoystickY), 0, 100, 0, 100);
        int rightMotorSpeed = map(max(0, rightJoystickY), 0, 100, 0, 100);

        // Serial.print("Left:");
        // Serial.print(leftMotorSpeed);
        // Serial.print("R2:");
        // // Serial.print(rightMotorSpeed);
        // // Serial.print("\n");
        //  Serial.println(R2);

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

          if(rightJoystickX>0){ // turn and move forward
            if(rightJoystickY>0){
              *ptr_speedEsc2 = rightJoystickX*(1.0-float(rightMotorSpeed)/100.0);
              *ptr_speedEsc1 = -rightJoystickX;
            } else if(rightJoystickY<0){
              *ptr_speedEsc2 = rightJoystickX;
              *ptr_speedEsc1 = -rightJoystickX*(1.0-float(leftMotorSpeed)/100.0);
            }
          } else if(rightJoystickX==0){ // turn in place
            if(rightJoystickY<0){
              *ptr_speedEsc2 = 500*(float(leftMotorSpeed)/100.0);
              *ptr_speedEsc1 = 500*(float(leftMotorSpeed)/100.0);
            } else if(rightJoystickY>0){
              *ptr_speedEsc2 = -500*(float(rightMotorSpeed)/100.0);
              *ptr_speedEsc1 = -500*(float(rightMotorSpeed)/100.0);
            }
          }

        } else if(R2 == 0 && rightJoystickX != 0){ // forward
          *ptr_speedEsc1 = constrain(rightJoystickX + yaw_pid_fw, -700, 700);
          *ptr_speedEsc2 = -constrain(rightJoystickX + yaw_pid_fw, -700, 700);
        } else if(R2 != 0){ // backward
          *ptr_speedEsc1 = constrain(-R2*7*7 + yaw_pid_fw, -700, 700);
          *ptr_speedEsc2 = constrain(R2*7*7 + yaw_pid_fw,-700, 700);
        } else { // hold yaw angle
          *ptr_speedEsc1 = -yaw_pid;
          *ptr_speedEsc2 = -yaw_pid;
        }


        /******************* CAPTURE CONTROL *******************/
        if(last_toggle == 1 && X ==0 ){
          catch_flag = !catch_flag;
        }
              
        last_toggle = X;

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