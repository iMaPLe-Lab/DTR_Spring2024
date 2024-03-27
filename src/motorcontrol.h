
//////////// Set up motors and shields ////////////
#include <Arduino.h>
#include "DShotESC.h"


#define PEAKSPEED 50
#define SINE_DURATION 10000.f //duration of the full cycle, in millis

DShotESC esc0;
DShotESC esc1;

int *ptr_throttle;

// Adafruit_MotorShield AFMStop(0x61); // Rightmost jumper closed
// Adafruit_MotorShield AFMS(0x61); 
// Adafruit_DCMotor *motorRight = AFMS.getMotor(4);
// Adafruit_DCMotor *motorLeft = AFMS.getMotor(3);
// Adafruit_DCMotor *motorUp = AFMS.getMotor(1);

//////////// Define const ////////////
// #define CATCHSPEED 200
// #define HOLDSPEED 150

// void motorSetup() {
//   Serial.println("Adafruit Motorshield v2 - DC Motor test!");

//   if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
//   // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
//     Serial.println("Could not find Motor Shield. Check wiring.");
//     while (1);
//   }
//   Serial.println("Motor Shield found.");

//   // if (!AFMSbot.begin()) {         // create with the default frequency 1.6KHz
//   // // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
//   //   Serial.println("Could not find Motor Shield 2. Check wiring.");
//   //   while (1);
//   // }
//   // Serial.println("Motor Shield 2 found.");

//   Serial.println("Motor Setup Completed!");
// }


void escUpdateTask(void * parameter) {
  for (;;) {
    esc0.sendThrottle3D(200);
    esc1.sendThrottle3D(400);
    vTaskDelay(pdMS_TO_TICKS(10)); // Delay for 100ms (10Hz)
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

  delay(1000);


  // right motor
  pinMode(GPIO_NUM_37, OUTPUT);
	esc1.install(GPIO_NUM_37, RMT_CHANNEL_1);
	esc1.init();
  esc1.throttleArm(1700);
	esc1.setReversed(true);
	esc1.set3DMode(true);


  // left motor


  // Create the ESC update task
  xTaskCreate(escUpdateTask, "ESC Update Task", 2048, NULL, 1, NULL);
  
}

void runMotors(int powerLeft, int powerRight, int powerUp) {
	
	esc0.sendThrottle3D(constrain(powerUp, -900, 900));
	esc1.sendThrottle3D(constrain(powerRight, -900, 900));
  // esc2.sendThrottle3D(constrain(powerLeft, -900, 900));
	
}


// void runMotors(int powerLeft, int powerRight, int powerUp){

//   if (powerLeft > 0){
//     motorLeft->run(BACKWARD);
//   } else {
//     motorLeft->run(FORWARD);
//   }

//   if (powerRight > 0){
//     motorRight->run(FORWARD);
//   } else {
//     motorRight->run(BACKWARD);
//   }

//   if (powerUp > 0){
//     motorUp->run(BACKWARD);
//   } else {
//     motorUp->run(FORWARD);
//   }

//   motorLeft->setSpeed(constrain(abs(powerLeft), 0, 250));
//   motorRight->setSpeed(constrain(abs(powerRight), 0, 250));
//   motorUp->setSpeed(constrain(abs(powerUp), 0, 250));

// }
