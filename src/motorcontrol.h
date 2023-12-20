
//////////// Set up motors and shields ////////////


// Adafruit_MotorShield AFMStop(0x61); // Rightmost jumper closed
Adafruit_MotorShield AFMS(0x61); 
Adafruit_DCMotor *motorRight = AFMS.getMotor(4);
Adafruit_DCMotor *motorLeft = AFMS.getMotor(3);
Adafruit_DCMotor *motorUp = AFMS.getMotor(1);

//////////// Define const ////////////
#define CATCHSPEED 200
#define HOLDSPEED 150

void motorSetup() {
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  // if (!AFMSbot.begin()) {         // create with the default frequency 1.6KHz
  // // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
  //   Serial.println("Could not find Motor Shield 2. Check wiring.");
  //   while (1);
  // }
  // Serial.println("Motor Shield 2 found.");

  Serial.println("Motor Setup Completed!");
}


void runMotors(int powerLeft, int powerRight, int powerUp){

  if (powerLeft > 0){
    motorLeft->run(BACKWARD);
  } else {
    motorLeft->run(FORWARD);
  }

  if (powerRight > 0){
    motorRight->run(FORWARD);
  } else {
    motorRight->run(BACKWARD);
  }

  if (powerUp > 0){
    motorUp->run(BACKWARD);
  } else {
    motorUp->run(FORWARD);
  }

  motorLeft->setSpeed(constrain(abs(powerLeft), 0, 250));
  motorRight->setSpeed(constrain(abs(powerRight), 0, 250));
  motorUp->setSpeed(constrain(abs(powerUp), 0, 250));

}
