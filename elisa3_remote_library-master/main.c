#include <stdio.h>
#include <sys/types.h>
#include <time.h>
#include "elisa3-lib.h"
#ifdef _WIN32
    #include "windows.h"
#endif

#define OBSTACLE_THR 50

unsigned char updateRGB(char *red, char *green, char *blue) {
    static unsigned int i=0;
    unsigned int rndNum;

    i = (i+1)%65000;  // use to change the rgb leds
    if(i==0) {
        rndNum = rand()%400;
        if(rndNum < 100) {
            *red = rand()%100;
            *green = rand()%100;
            *blue = 0;
        } else if(rndNum < 200) {
            *red = rand()%100;
            *green = 0;
            *blue = rand()%100;
        } else if(rndNum < 300) {
            *red = 0;
            *green = rand()%100;
            *blue = rand()%100;
        } else {
            *red = rand()%100;
            *green = rand()%100;
            *blue = rand()%100;
        }
        return 1;
    }
    return 0;
}

void avoidObstacles(unsigned int *prox, char *left, char *right) {
    int rightProxSum=0, leftProxSum=0;	// sum of proximity values on right or left

    // obstacle avoidance using the 3 front proximity sensors
    rightProxSum = prox[0]/2 + prox[1];
    leftProxSum = prox[0]/2 + prox[7];

    rightProxSum /= 5;                 // scale the sum to have a moderate impact on the velocity
    leftProxSum /= 5;
    if(rightProxSum > 60) {             // velocity of the motors set to be from -30 to +30
        rightProxSum = 60;
    }
    if(leftProxSum > 60) {
        leftProxSum = 60;
    }
    *right = 30-leftProxSum;     // set the speed to the motors
    *left = 30-rightProxSum;

}

int main(void) {
    // Hardcoded array of robot addresses
    int robotAddresses[] = {4075, 4050,4065,4060}; // Example addresses
    int numRobots = sizeof(robotAddresses) / sizeof(robotAddresses[0]);

    // Rest of your variables...
    unsigned int robProx[8] = {0};
    char robLSpeed=0, robRSpeed=0;
    char robRedLed=0, robGreenLed=0, robBlueLed=0;

    srand(time(NULL));

    // Initialize the communication with the robots
    startCommunication(robotAddresses, numRobots);

    while(1) {
        for(int i = 0; i < numRobots; i++) {
            int currentRobot = robotAddresses[i];

            // Get proximity, avoid obstacles, and set speeds for each robot
            getAllProximity(currentRobot, robProx);
            avoidObstacles(robProx, &robLSpeed, &robRSpeed);
            setLeftSpeed(currentRobot, robLSpeed);
            setRightSpeed(currentRobot, robRSpeed);

            // Update and set RGB LEDs for each robot
            if(updateRGB(&robRedLed, &robGreenLed, &robBlueLed)) {
                setRed(currentRobot, robRedLed);
                setGreen(currentRobot, robGreenLed);
                setBlue(currentRobot, robBlueLed);
            }
        }
    }

    stopCommunication();
    return 0;
}