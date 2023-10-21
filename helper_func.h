#ifndef HELPER_FUNC_H
#define HELPER_FUNC_H

#include <math.h>
#include <vector>
#include <random>
#include <iostream>
#include "robot_defs.h"

extern std::random_device rd;
extern std::mt19937 randEngine;

double gaussianRandom(double mean, double variance);

double pdfGaussian(double mu, double sigma, double x);

void globalToRobotFrame(RobotState& robotState, double globalX, double globalY, double& localX, double& localY);

double measurementProbability(RobotState& particle, std::vector<MarkerObservation>& observations, 
                              std::vector<FieldLocation>& landmarks, RobotParams& rParams);

void updateWeights(std::vector<double>& weights, std::vector<RobotState>& particles, std::vector<MarkerObservation>& observations, 
                   std::vector<FieldLocation>& landmarks, RobotParams& rParams, double& maxWeight, double& Neff);

void updateCurrentEstimate(RobotState& currentEstimate, std::vector<RobotState>& particles, std::vector<double>& weights);


#endif // HELPER_FUNC_H