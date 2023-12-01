
// Global variables for the PID
double integral_yaw = 0;
double integral_altitude = 0;
double previous_error_yaw = 0;
double previous_error_altitude = 0;

// PID constants
const double kp_yaw = 3.0;
const double ki_yaw = 0.001;
const double kd_yaw = 0.05;
const double kp_alt = 1000.0;
const double ki_alt = 0.1;
const double kd_alt = 0.5;

double computePID_yaw(double goal_yaw, double current_yaw) {

  double error_yaw = goal_yaw - current_yaw;
  integral_yaw += error_yaw;

  double derivative_yaw = error_yaw - previous_error_yaw;
  previous_error_yaw = error_yaw;

  double output_yaw = kp_yaw * error_yaw + ki_yaw * integral_yaw + kd_yaw * derivative_yaw;

  return output_yaw;
}


double computePID_altitude(double goal_altitude, double current_altitude) {

  double error_altitude = goal_altitude - current_altitude;
  integral_altitude += error_altitude;

  double derivative_altitude = error_altitude - previous_error_altitude;
  previous_error_altitude = error_altitude;

  double output_altitude = kp_alt * error_altitude + ki_alt * integral_altitude + kd_alt * derivative_altitude;

  return output_altitude;
}
