#ifndef DShotController_h
#define DShotController_h

#include <Arduino.h>
#include "DShotESC.h"

class DShotController {
public:
  DShotESC esc0;
  DShotESC esc1;
  DShotESC esc2;
  void motorSetUp(); // Initializes ESCs and possibly other setup tasks
};

#endif
