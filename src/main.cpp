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

Servo myservo;
int servoPin = 9;
#define PEAKSPEED 500
#define SINE_DURATION 10000.f //duration of the full cycle, in millis
int pos = 0;  

void setReports(sh2_SensorId_t reportType, long report_interval);
void sensorSetup();

//////// Define global variables ////////////
int leftJoystickX   = 0; // (-100 - 100) left X axis
int leftJoystickY   = 0; // (-100 - 100) left Y axis
int leftThrottle    = 0; // backward
int rightThrottle   = 0; // forward
int Brakebutton     = 0;
int rightJoystickX  = 0; // (-100 - 100) right X axis
int rightJoystickY  = 0; // (-100 - 100) right Y axis
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


double target_alt = 150; // cm
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
bool is_manual_alt = false;

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
  Serial1.begin(115200);

  motorSetup();
  sensorSetup();

  myservo.attach(servoPin, 1000, 2000); // attaches the servo on pin 18 to the servo object

  //////////// Wifi setup ////////////
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial1.println("Connecting to WiFi...");
    Serial.println("Connecting to WiFi...");
  }
  Serial1.println("Connected to WiFi");
  Serial1.println(WiFi.localIP());

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
        readTOF();
        // Serial1.printf( "Alt: %04icm ", tfDist);
        // Serial1.print(", Target Alt: ");
        // Serial1.println(target_alt);

        // Serial.printf( "Alt: %04icm ", tfDist);
        // Serial.print(", Target Alt: ");
        // Serial.println(target_alt);

        double alt_pid = 200+constrain(computePID_altitude(target_alt, tfDist), 0, 500);
        // Serial1.print("Altitude PID value: ");
        // Serial1.println(alt_pid);

        // Serial.print("Altitude PID value: ");
        // Serial.println(alt_pid);

        if (abs(leftJoystickY) > 0) {
          *ptr_speedEscAltitude = leftJoystickY*6; // manual up and down
          is_manual_alt = true;
        } else {
          *ptr_speedEscAltitude = alt_pid; // hold altitude with PID
        }

        if (is_manual_alt && abs(leftJoystickY) < 2) { // Just stopped changing altitude
          is_manual_alt = false;
          target_alt = tfDist; // set alt position to hold
        }


        /******************* DIRECTION/YAW CONTROL *******************/
        readIMU();
        Serial1.print("Yaw: ");
        Serial1.print(ypr.yaw);
        Serial1.print(", Target Yaw: ");
        Serial1.println(target_yaw);

        int leftMotorSpeed = map(max(0, -rightJoystickY), 0, 100, 0, 100);
        int rightMotorSpeed = map(max(0, rightJoystickY), 0, 100, 0, 100);

        if (is_turning && abs(rightJoystickY) < 5) { // Just stopped turning
          is_turning = false;
          target_yaw = ypr.yaw; // set yaw position to hold
        }
        
        double yaw_pid = constrain(computePID_yaw(target_yaw, ypr.yaw), -350, 350);
        double yaw_pid_fw = constrain(computePID_yaw_fw(target_yaw, ypr.yaw), -300, 300);

        Serial1.print("Yaw PID: ");
        Serial1.print(yaw_pid);
        Serial1.print(", Yaw PID FWD: ");
        Serial1.println(yaw_pid_fw);


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
          *ptr_speedEscLeft =  constrain(leftThrottle*7+yaw_pid_fw, -700, 700);
          *ptr_speedEscRight = -constrain(leftThrottle*7+yaw_pid_fw, -700, 700);
        } else if(rightThrottle != 0){ // forward?
          
          *ptr_speedEscLeft = -constrain(rightThrottle*7+yaw_pid_fw, -700, 700);
          *ptr_speedEscRight =  constrain(rightThrottle*7+yaw_pid_fw,-700, 700);
        } 
        else if(R2==0 && rightJoystickX == 0 ){ // hold yaw angle
          *ptr_speedEscLeft = -yaw_pid;
          *ptr_speedEscRight = -yaw_pid;
        }

        Serial1.print("Altitude Power: ");
        Serial1.print(speedEscAltitude);
        Serial1.print(", Left Power: ");
        Serial1.print(speedEscLeft);
        Serial1.print(", Right Power: ");
        Serial1.println(speedEscRight);


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
    Serial1.println("Client disconnected");
    Serial.println("Client disconnected");
  }
}