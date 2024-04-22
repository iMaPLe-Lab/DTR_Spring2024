#include <TFMPI2C.h>  // TFMini-Plus I2C Library v1.7.3
TFMPI2C tfmP;         // Create a TFMini-Plus I2C object

//////////// IMU Constants ////////////
#define BNO08X_RESET -1
#define SDA0_Pin 41   // select ESP32  I2C pins
#define SCL0_Pin 40

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;
double target_yaw = 0;

Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

int16_t tfDist = 0;       // Distance to object in centimeters
int16_t tfFlux = 0;       // Signal strength or quality of return signal
int16_t tfTemp = 0;       // Internal temperature of Lidar sensor chip

#ifdef FAST_MODE
  // Top frequency is reported to be 1000Hz (but freq is somewhat variable)
  sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
  long reportIntervalUs = 2000;
#else
  // Top frequency is about 250Hz but this report is more accurate
  sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
  long reportIntervalUs = 5000;
#endif

void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
  Serial1.println("Setting desired reports");
  if (! bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
    Serial1.println("Could not enable stabilized remote vector");
  }
}


void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
      ypr->yaw *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= RAD_TO_DEG;
    }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}
 
void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}


void readIMU(){
  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    Serial1.print("sensor was reset ");
    setReports(reportType, reportIntervalUs);
  }
  
  if (bno08x.getSensorEvent(&sensorValue)) {
    // in this demo only one report type will be received depending on FAST_MODE define (above)
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
      case SH2_GYRO_INTEGRATED_RV:
        // faster (more noise?)
        quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
        break;
    }
    static long last = 0;
    long now = micros();
    last = now;
  }  
//   return ypr;
}

void readTOF(){
  tfmP.getData(tfDist, tfFlux, tfTemp); // Get a frame of data
    // if( tfmP.status == TFMP_READY)         // If no error...
    // {
    //     Serial.printf( "Dist:%04icm ", tfDist);   // display distance,
    //     Serial.printf( "Flux:%05i ", tfFlux);     // display signal strength/quality,
    //     Serial.printf( "Temp:%2i%s", tfTemp, "°C" );   // display temperature,
    //     Serial.printf( "\n");                     // end-of-line.
    // }
    // else
    // {
    //     tfmP.printFrame();                 // Display error and data frame
    //     if( tfmP.status == TFMP_I2CWRITE)  // If I2C error...
    //     {
    //         // tfmP.recoverI2CBus();          // recover hung bus.
    //         Serial.println("Couldn't connect to TOF");
    //     }
    // }
}

void sensorSetup() {

  //////////// IMU setup ////////////
  Wire1.begin(SDA0_Pin, SCL0_Pin);


  if (!bno08x.begin_I2C(0x4A, &Wire1)) {
    Serial.println("Failed to find BNO08x chip");
    Serial1.println("Failed to find BNO08x chip");
    while (1) yield();
  }
  Serial.println("BNO08x ok ");
  Serial1.println("BNO08x ok ");

  setReports(reportType, reportIntervalUs);

  readIMU();
  delay(250);

  readIMU();
  target_yaw = ypr.yaw;
  Serial.print("Initial target yaw: ");
  Serial.println(target_yaw);
  Serial1.print("Initial target yaw: ");
  Serial1.println(target_yaw);

  //////////// TOF setup ////////////
  Serial.printf( "System reset: ");
  Serial1.printf( "System reset: ");
  if( tfmP.sendCommand( SOFT_RESET, 0))
  {
    Serial.printf( "passed.\r\n");
    Serial1.printf( "passed.\r\n");
  }
  else tfmP.printReply();

  Serial.printf( "Enable Output: ");
  if( tfmP.sendCommand( ENABLE_OUTPUT, 0))
  {
    Serial.printf( "passed.\r\n");
    Serial1.printf( "passed.\r\n");
  }
  else tfmP.printReply();

  delay(250);

  readTOF();

}