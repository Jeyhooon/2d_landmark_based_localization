#ifndef HELPER_FUNC_H
#define HELPER_FUNC_H

#include <math.h>
#include <vector>
#include <random>
#include <iostream>
#include "robot_defs.h"

extern std::random_device rd;
extern std::mt19937 randEngine;

void clipOrientation(double& theta);

double gaussianRandom(double mean, double variance);

double pdfGaussian(double mu, double sigma, double x);

void globalToRobotFrame(RobotState& robotState, double globalX, double globalY, double& localX, double& localY);

double obsProbability(RobotState& particle, std::vector<MarkerObservation>& observations, 
                              std::vector<FieldLocation>& landmarks, RobotParams& rParams);

void updateWeights(std::vector<double>& weights, std::vector<RobotState>& particles, std::vector<MarkerObservation>& observations, 
                   std::vector<FieldLocation>& landmarks, RobotParams& rParams, double& Neff, double& maxWeight);

void updateCurrentEstimate(RobotState& currentEstimate, std::vector<RobotState>& particles);

void resampleParticles(std::vector<RobotState>& particles, std::vector<double>& weights, double maxWeight);

void sampleRandomParticles(std::vector<RobotState>& particles, int N);

void sampleCircular(std::vector<MarkerObservation>& observations, std::vector<RobotState>& particles, 
                     std::vector<FieldLocation>& landmarks, RobotParams& rParams, int N);

#endif