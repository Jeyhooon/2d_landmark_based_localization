#ifndef _MAIN_H_
#define _MAIN_H_

#include <vector>
#include "robot_defs.h"
#include "helper_func.h"

// Sets current robot position estimate
void getRobotPositionEstimate(RobotState& estimatePosn);

// Robot motion update
void motionUpdate(RobotState delta);

// Landmark sensor update
void sensorUpdate(std::vector<MarkerObservation> observations);

// Initialization and UI methods
void myinit(RobotState robotState, RobotParams robotParams, 
            FieldLocation markerLocations[NUM_LANDMARKS]);
void mydisplay();
int  mykeyboard(unsigned char key);

#endif
