
#include "WiFi.h"
#include "Arduino.h"
#include "Wire.h"
#include "Adafruit_VL53L1X.h"
#include "Adafruit_BNO08x.h"
#include "Adafruit_MotorShield.h"
#include "motorcontrol.h"
#include "pid.h"
#include "sensors.h"
#include "wificonnection.h"
#include "setup.h"

// Define custom functions
void motorSetup();
void runMotors(int powerLeft, int powerRight, int powerUp);
double angle_difference(double angle1, double angle2);
double computePID_yaw(double goal_yaw, double current_yaw);
double computePID_yaw_fw(double goal_yaw, double current_yaw);
double computePID_altitude(double goal_altitude, double current_altitude);
String readInteger(WiFiClient& client);
void processData(String data, int* ptrLY, int* ptrRX, int* ptrTru, int* ptrBreak, int* ptrCat, int* ptrAtt);
void setReports(sh2_SensorId_t reportType, long report_interval);
void sensorSetup();


//////// Define global variables ////////////
int leftJoystickX   = 0;  // (-511 - 512) left X axis
int leftJoystickY   = 0;  // (-511 - 512) left Y axis
int Throttlebutton  = 0; // for altitude increase (0-1023)
int Brakebutton     = 0; // for altitude decrease (0-1023)
int rightJoystickX  = 0; // (-511 - 512) right X axis
int rightJoystickY  = 0; // (-511 - 512) right Y axis
int catchs          = 0;
int attKill         = 0;

int* ptrLeftJoystickX   = &leftJoystickX;
int* ptrLeftJoystickY   = &leftJoystickY;
int* ptrThrottlebutton  = &Throttlebutton;
int* ptrBrakebutton     = &Brakebutton;
int* ptrRightJoystickX  = &rightJoystickX;
int* ptrRightJoystickY  = &rightJoystickY;
int* ptrCatchs          = &catchs;
int* ptrAttKill         = &attKill;
unsigned long lastUpdate = 0; // Stores the last update time
const unsigned long updateInterval = 3000; // Update interval in milliseconds (500ms)

double target_yaw = 0;
double target_alt = 0;
int powerLeft = 0;
int powerRight = 0;
int powerUp = 0;
int forward = 0;
int turn = 0;
int up = 0;


// Arduino loop function. Runs in CPU 1
void loop() {

  // delay(10);
  WiFiClient client = server.available();
  if (client) {
    Serial.println("Client connected");
    while (client.connected()) {
      if (client.available()) {
        char typeIdentifier = client.read();
        if (typeIdentifier == 'I') {
          String result = readInteger(client);
          // processData(result, ptrRightJoystickX, ptrLeftJoystickY, ptrThrust, ptrBreak, ptrcatchs, ptrstable);
          processData(result, ptrLeftJoystickY, ptrRightJoystickX, ptrThrottlebutton, ptrBrakebutton, ptrCatchs, ptrAttKill);
          Serial.print(leftJoystickY);
          Serial.print(", ");
          Serial.print(rightJoystickX);
          Serial.print(", ");
          Serial.print(Throttlebutton);
          Serial.print(", ");
          Serial.print(Brakebutton);
          Serial.print(", ");
          Serial.print(catchs);
          Serial.print(", ");
          Serial.println(attKill);

          // Altitude control
          if (Throttlebutton > 30){
            target_alt += Throttlebutton/600;
            target_alt = constrain(target_alt, 0, 4000); // UPDATE based on sensor
          }
          if (Brakebutton > 30){
            target_alt -= Brakebutton/600;
          }      
          target_alt = constrain(target_alt, 0, 4000); // UPDATE based on sensor

          // Yaw control
          if (abs(rightJoystickX) > 30){ // actively turning, so don't hold position with IMU
            turn = rightJoystickX/2;
            target_yaw = readIMU();
          }
          else if (abs(rightJoystickX) > 1){ // just released control input, set target yaw to hold
            target_yaw = readIMU();
            turn = computePID_yaw(target_yaw, readIMU());
          }
          else { // hold target yaw with PID
            turn = computePID_yaw(target_yaw, readIMU());
          }
          turn = constrain(turn, -200, 200);

          // Direction control
          if (abs(leftJoystickY) > 30){
            forward = leftJoystickY/2;
          } else if (abs(leftJoystickY) > 1){
            forward = 0;
          }
          forward = constrain(forward, -200, 200);

          // Kill altitude
          if (attKill){
            target_alt = 0;
          }

          // Run motors
          powerLeft  = forward + turn;
          powerRight = forward - turn;
          powerUp    = computePID_altitude(target_alt, readTOF());
          runMotors(powerLeft, powerRight, powerUp);
     

///////////////////////////////////////////////////////////////
        }
      }
    }
    client.stop();
    Serial.println("Client disconnected");
  }
}

