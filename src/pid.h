
// Global variables for the PID
double integral_yaw = 0;
double integral_altitude = 0;
double previous_error_yaw = 0;
double previous_error_altitude = 0;

// PID constants
double kp = 2.0;
double ki = 0.0;
double kd = 0.05;

double kp_alt = 10;
double ki_alt = 0;
double kd_alt = 0;

const double kp_f = 1;
const double ki_f = 0;
const double kd_f = 0;


void updatePID(String data) {
  // Find positions of each parameter in the received data
  int pIndex = data.indexOf('P');
  int iIndex = data.indexOf('I');
  int dIndex = data.indexOf('D');
  int endIndex = data.length();
  // Extract each parameter's value as substring
  String pValue = data.substring(pIndex + 2, iIndex - 1);
  String iValue = data.substring(iIndex + 2, dIndex - 1);
  String dValue = data.substring(dIndex + 2, endIndex - 1);
  // Convert string to double
  kp = pValue.toDouble();
  ki = iValue.toDouble();
  kd = dValue.toDouble();
  // Debugging: Output updated values
  Serial1.print("Updated Pk: ");
  Serial1.println(kp);
  Serial1.print("Updated Ik: ");
  Serial1.println(ki);
  Serial1.print("Updated Dk: ");
  Serial1.println(kd);
}

double angle_difference(double angle1, double angle2) { //angle1 current , angle2 target 
    double difference = angle2 - angle1;
    if (difference > 180) {
        difference -= 360;
    } else if (difference < -180) {
        difference += 360;
    }
   
    return difference;
}

double computePID_yaw(double goal_yaw, double current_yaw) {
  double error_yaw = angle_difference(current_yaw, goal_yaw);
  Serial1.println(error_yaw);
  integral_yaw += error_yaw;
  double derivative_yaw = error_yaw - previous_error_yaw;
  previous_error_yaw = error_yaw;
  double output_yaw = kp * error_yaw + ki * integral_yaw + kd * derivative_yaw;
  return output_yaw;
}

double computePID_yaw_fw(double goal_yaw, double current_yaw) {
  double error_yaw = angle_difference(current_yaw, goal_yaw);
  integral_yaw += error_yaw;
  double derivative_yaw = error_yaw - previous_error_yaw;
  previous_error_yaw = error_yaw;
  double output_yaw = kp_f * error_yaw + ki_f * integral_yaw + kd_f * derivative_yaw;
  return output_yaw;
}

double computePID_altitude(double goal_altitude, double current_altitude) {

  double error_altitude = goal_altitude - current_altitude; //cm 
  integral_altitude += error_altitude;

  double derivative_altitude = error_altitude - previous_error_altitude;
  previous_error_altitude = error_altitude;

  double output_altitude = kp_alt * error_altitude + ki_alt * integral_altitude + kd_alt * derivative_altitude;

  return output_altitude;
}