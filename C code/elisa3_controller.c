#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#define TIME_STEP 64
#define MAX_SPEED 6.28

// Function prototypes
int getRank(const char* robotName);
double getDistance(double x1, double y1, double x2, double y2);
double checkAngle(double x1, double y1, double x2, double y2);
void moveForward(WbDeviceTag leftMotor, WbDeviceTag rightMotor, double setSpeed);
void rotateLeft(WbDeviceTag leftMotor, WbDeviceTag rightMotor, double setSpeed);
void rotateRight(WbDeviceTag leftMotor, WbDeviceTag rightMotor, double setSpeed);
void stop(WbDeviceTag leftMotor, WbDeviceTag rightMotor);

int main(int argc, char **argv) {
  wb_robot_init();

  WbDeviceTag leftMotor = wb_robot_get_device("left wheel motor");
  WbDeviceTag rightMotor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(leftMotor, INFINITY);
  wb_motor_set_position(rightMotor, INFINITY);
  wb_motor_set_velocity(leftMotor, 0.0);
  wb_motor_set_velocity(rightMotor, 0.0);

  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);

  // Variable declarations
  bool lookingHeading = false;
  double setSpeed = 0.25;
  double robotHeading = 0.0;

  // Initialize variables
  double xcoords[] = {0, 0, 0, 0, 0, 0, -0.5, -1, 0.5, 1};
  double ycoords[] = {-1.5, -1, -0.5, 0, 0.5, 1, 1, 1, 1, 1};

  // Main loop
  while (wb_robot_step(TIME_STEP) != -1) {
    const double *gpsValues = wb_gps_get_values(gps);
    const char* robotName = wb_robot_get_name();
    double x1 = gpsValues[0];
    double y1 = gpsValues[1];

    // Compute desired heading
    double x2 = xcoords[getRank(robotName)-1];
    double y2 = ycoords[getRank(robotName)-1];
    double desiredHeading = checkAngle(x1, y1, x2, y2);

    // Obtain more accurate heading estimation
    if (!lookingHeading) {
      moveForward(leftMotor, rightMotor, setSpeed);
      wb_robot_step(TIME_STEP);

      const double *gpsValues2 = wb_gps_get_values(gps);
      double x3 = gpsValues2[0];
      double y3 = gpsValues2[1];

      robotHeading = checkAngle(x1, y1, x3, y3);
      rotateLeft(leftMotor, rightMotor, setSpeed);
      wb_robot_step(TIME_STEP);
    }

    double error = fabs(robotHeading - desiredHeading);
    
    if (error < 0.1) {
      lookingHeading = true;
      moveForward(leftMotor, rightMotor, setSpeed);
    }
    double distance = getDistance(x1, y1, x2, y2);

    // Stop if in place
    if (distance < 0.1) {
      stop(leftMotor, rightMotor);
      //printf("In place\n");
    }

    // Update the position if error is greater than 0.1
    if (error > 0.1) {
      lookingHeading = false;
      //printf("Heading false\n");
    }
  }

  wb_robot_cleanup();

  return 0;
}

// Function definitions
int getRank(const char *robotName)
{
  int rank = 0 ;

  if(strlen(robotName) >= 2){
    const char* robotNum = &robotName[strlen(robotName)-2];
    rank = atoi(robotNum);
  }
  return rank;
}

double getDistance(double x1, double y1, double x2, double y2) {
  return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

double checkAngle(double x1, double y1, double x2, double y2) {
  return atan2(y2 - y1, x2 - x1);
}

void moveForward(WbDeviceTag leftMotor, WbDeviceTag rightMotor, double setSpeed)
{
  wb_motor_set_velocity(leftMotor, MAX_SPEED * setSpeed);
  wb_motor_set_velocity(rightMotor, MAX_SPEED * setSpeed);
}

void rotateLeft(WbDeviceTag leftMotor, WbDeviceTag rightMotor, double setSpeed)
{
  wb_motor_set_velocity(leftMotor, -MAX_SPEED * setSpeed);
  wb_motor_set_velocity(rightMotor, MAX_SPEED * setSpeed);
}

void rotateRight(WbDeviceTag leftMotor, WbDeviceTag rightMotor, double setSpeed)
{
  wb_motor_set_velocity(leftMotor, MAX_SPEED * setSpeed);
  wb_motor_set_velocity(rightMotor, -MAX_SPEED * setSpeed);
}

void stop(WbDeviceTag leftMotor, WbDeviceTag rightMotor)
{
  wb_motor_set_velocity(leftMotor, 0.0);
  wb_motor_set_velocity(rightMotor, 0.0);
}
