#include "helper_func.h"

std::random_device rd;
std::mt19937 randEngine(rd());
std::vector<RobotState> newParticles(MAX_NUM_PARTICLES);
std::normal_distribution<double> resampleNoiseDist(0.0, 0.01);
std::uniform_real_distribution<double> resampleWheelDist(0, 1.0);


double gaussianRandom(double mean, double variance)
{
    // Gaussian random
    std::normal_distribution<double> gauss_dist(mean, variance);
    return gauss_dist(randEngine);
}

double pdfGaussian(double mu, double sigma, double x)
{
    // Probability of x for 1-dim Gaussian with mean mu and stddev sigma
    return exp(-(pow((mu - x), 2)) / (pow(sigma, 2)) / 2.0) / sqrt(2.0 * M_PI * (pow(sigma, 2)));
}

void globalToRobotFrame(RobotState& robotState, double globalX, double globalY, double& localX, double& localY)
{
    // This converts a point from the global reference frame to the robot's local reference frame
    // (useful for calculating the landmark's location in particles local reference frame)
    localX = (globalX - robotState.x) * cos(robotState.theta) + (globalY - robotState.y) * sin(robotState.theta);
    localY = -(globalX - robotState.x) * sin(robotState.theta) + (globalY - robotState.y) * cos(robotState.theta);
}

double measurementProbability(RobotState& particle, std::vector<MarkerObservation>& observations, 
                              std::vector<FieldLocation>& landmarks, RobotParams& rParams)
{
    // Calculates how likely measurements are for the given particle
    double prob = 1.0;
    double dist;
    double theta;

    for(const auto& obs : observations)
    {
        double localObsX, localObsY;    // local coordinates of the landmark from particles perspective
        globalToRobotFrame(particle, landmarks[obs.markerIndex].x, landmarks[obs.markerIndex].y, localObsX, localObsY);
        
        dist = sqrt(pow(localObsX, 2) + pow(localObsY, 2));
        prob *= pdfGaussian(obs.distance, rParams.sensor_noise_distance, dist);
        
        theta = atan2(localObsY, localObsX);    // values in [-pi, pi] interval
        prob *= pdfGaussian(obs.orientation, rParams.sensor_noise_orientation, theta);
    }

    return prob;
}

void updateWeights(std::vector<double>& weights, std::vector<RobotState>& particles, std::vector<MarkerObservation>& observations, 
                   std::vector<FieldLocation>& landmarks, RobotParams& rParams, double& Neff, double& maxWeight)
{
    // calculate particle importance weights
    weights.clear();
    int N = particles.size();
    std::cout << "Number of particles: " << N << std::endl;
    double sumWeights = 0;

    for (int i = 0; i < N; i++)
    {
        weights.push_back(measurementProbability(particles[i], observations, landmarks, rParams));
        sumWeights += weights[i];
    }   

    // normalize weights
    maxWeight = -1;
    double sumSquaredWeights = 0.0;
    for (int i = 0; i < N; i++)
    {
        weights[i] /= sumWeights;
        if (weights[i] > maxWeight) maxWeight = weights[i];
        sumSquaredWeights += pow(weights[i], 2);
    }
    std::cout << "maxWeight: " << maxWeight << std::endl;

    // calculate effective number of particles
    Neff = 1.0 / sumSquaredWeights;
    std::cout << "Neff: " << Neff << std::endl;
}

void updateCurrentEstimate(RobotState& currentEstimate, std::vector<RobotState>& particles, std::vector<double>& weights)
{
    // only update the current estimate if there are particles
    if(!particles.empty())
    {
        // use mean values of particles as current estimate
        double sumX = 0, sumY = 0, sumThetaX = 0, sumThetaY = 0;
        for (auto& p : particles) {
            sumX += p.x;
            sumY += p.y;
            sumThetaX += cos(p.theta);
            sumThetaY += sin(p.theta);
        }

        double numParticles = static_cast<double>(particles.size());
        currentEstimate.x = sumX/numParticles*(1 - GAMMA) + currentEstimate.x * GAMMA;
        currentEstimate.y = sumY/numParticles*(1 - GAMMA) + currentEstimate.y * GAMMA;
        currentEstimate.theta = atan2(sumThetaY/numParticles, sumThetaX/numParticles)*(1 - GAMMA) + currentEstimate.theta * GAMMA;
    }
}


void resampleParticles(std::vector<RobotState>& particles, std::vector<double>& weights, double Neff, double maxWeight)
{
    int N = particles.size();
    int N_resample = (1 - ALPHA) * N;
    int N_random = N - N_resample;
    
    std::cout << "Resampling particles..." << std::endl;
    //Resample the particles using Resampling Wheel technique
    newParticles.clear();

    double beta = 0;
    int index = resampleWheelDist(randEngine) * N;
    for (int i = 0; i < N; i++)
    {
        beta += resampleWheelDist(randEngine) * 2 * maxWeight;
        while(beta > weights[index])
        {
            beta -= weights[index];
            index = (index + 1) % N;
        }
        RobotState p;
        p.x = particles[index].x + resampleNoiseDist(randEngine);
        p.y = particles[index].y + resampleNoiseDist(randEngine);
        p.theta = particles[index].theta + resampleNoiseDist(randEngine);
        newParticles.push_back(p);
    }

    // Add random particles
    // std::uniform_real_distribution<double> randParticleDist(-1.0, 1.0);
    // for (int i = 0; i < N_random; ++i) {
    //     RobotState p;
    //     p.x = randParticleDist(randEngine) * (METERS_PER_PIXEL * (FIELD_LENGTH/2 + FIELD_OFFSET_X));
    //     p.y = randParticleDist(randEngine) * (METERS_PER_PIXEL * (FIELD_WIDTH/2 + FIELD_OFFSET_Y));
    //     p.theta = randParticleDist(randEngine) * M_PI;
    //     newParticles.push_back(p);
    // }

    particles = newParticles;
}