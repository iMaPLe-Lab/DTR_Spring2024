
// Global variables for the PID
double integral_yaw = 0;
double integral_altitude = 0;
double previous_error_yaw = 0;
double previous_error_altitude = 0;

// PID constants
const double kp = 1.5;
const double ki = 0;
const double kd = 0;

const double kp_alt = 0.5;
const double ki_alt = 0;
const double kd_alt = 0;

const double kp_f = 3.8;
const double ki_f = 0;
const double kd_f = 0;

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

  double error_altitude = goal_altitude - current_altitude;
  integral_altitude += error_altitude;

  double derivative_altitude = error_altitude - previous_error_altitude;
  previous_error_altitude = error_altitude;

  double output_altitude = kp_alt * error_altitude + ki_alt * integral_altitude + kd_alt * derivative_altitude;

  return output_altitude;
}
