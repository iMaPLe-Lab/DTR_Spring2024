#include "Dshot.h"
#define DSHOT_ARM_DURATION 1700
#define DSHOT600_FREQUENCY 600000UL
#define RMT_CLK_DIV 3
void DShotController::motorSetUp() {
      // Altitude motor setup
    pinMode(GPIO_NUM_35, OUTPUT);
	esc0.install(GPIO_NUM_35, RMT_CHANNEL_0);
	esc0.init();
    esc0.throttleArm(1700);
	esc0.setReversed(true);
	esc0.set3DMode(true);

    // delay(1700);


  // right motor
    pinMode(GPIO_NUM_37, OUTPUT);
	esc1.install(GPIO_NUM_37, RMT_CHANNEL_1);
	esc1.init();
    esc1.throttleArm(1700);
	esc1.setReversed(true);
	esc1.set3DMode(true);
// delay(1700);

      // right motor
    pinMode(GPIO_NUM_18, OUTPUT);
	esc2.install(GPIO_NUM_18, RMT_CHANNEL_2);
	esc2.init();
    esc2.throttleArm(1700);
	esc2.setReversed(true);
	esc2.set3DMode(true);

    // esc0.throttleArm(1700);
    // esc1.throttleArm(1700);
    // esc2.throttleArm(1700);
    delay(100);
    esc0.throttleArm(1700);
    esc1.throttleArm(1700);
    esc2.throttleArm(1700);

     
    // esc0.setReversed(true);
    
    // delay(1000);

    
    // Right motor setup
    // esc1.init();
  
    // esc1.setReversed(true);
    

//    delay(1000);
    // Left motor setup
    // // esc2.init();
    // esc2.throttleArm(DSHOT_ARM_DURATION);
    // esc1.throttleArm(DSHOT_ARM_DURATION);
    // esc0.throttleArm(DSHOT_ARM_DURATION);
    // esc2.setReversed(true);
    

 

    // delay(1000);

}




