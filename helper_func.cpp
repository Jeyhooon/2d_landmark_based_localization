#include "helper_func.h"
#include <assert.h>

std::random_device rd;
std::mt19937 randEngine(rd());
std::vector<RobotState> newParticles(MAX_NUM_PARTICLES);
std::normal_distribution<double> normDist(0.0, 1.0);
std::uniform_real_distribution<double> resampleWheelDist(0.0, 1.0);
std::uniform_real_distribution<double> uniformCandidateDist(-1.0, 1.0);

void clipOrientation(double& theta)
{
    // Ensure theta stays in [-pi, pi] interval
    if (theta > M_PI) theta -= 2 * M_PI;
    if (theta < -M_PI) theta += 2 * M_PI;
}

double gaussianRandom(double mean, double stddev)
{
    // Gaussian random number generator
    return normDist(randEngine)*stddev + mean;
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

double obsProbability(RobotState& particle, std::vector<MarkerObservation>& observations, 
                      std::vector<FieldLocation>& landmarks, RobotParams& rParams)
{
    // Calculates how likely measurements are from particle's perspective
    double prob = 1.0;
    double dist;
    double theta;

    for(const auto& obs : observations)
    {
        double localObsX, localObsY;    // local coordinates of the landmark from particles perspective
        globalToRobotFrame(particle, landmarks[obs.markerIndex].x, landmarks[obs.markerIndex].y, localObsX, localObsY);
        
        dist = sqrt(pow(localObsX, 2) + pow(localObsY, 2));
        prob *= pdfGaussian(dist, rParams.sensor_noise_distance, obs.distance);
        
        theta = atan2(localObsY, localObsX);    // values in [-pi, pi] interval
        prob *= pdfGaussian(theta, rParams.sensor_noise_orientation, obs.orientation);
    }

    return prob;
}

void updateWeights(std::vector<double>& weights, std::vector<RobotState>& particles, std::vector<MarkerObservation>& observations, 
                   std::vector<FieldLocation>& landmarks, RobotParams& rParams, double& Neff, double& maxWeight)
{
    // calculate particle importance weights
    weights.clear();
    int N = particles.size();
    double sumWeights = 0;

    for (int i = 0; i < N; i++)
    {
        weights.push_back(obsProbability(particles[i], observations, landmarks, rParams));
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

    // calculate effective number of particles
    Neff = 1.0 / sumSquaredWeights;
}

void updateCurrentEstimate(RobotState& currentEstimate, std::vector<RobotState>& particles)
{
    // only update the current estimate if there are particles
    if(particles.empty())
        return;

    double numParticles = static_cast<double>(particles.size());
    double sumX = 0, sumY = 0, sumThetaX = 0, sumThetaY = 0;
    for (auto& p : particles) {
        sumX += p.x;
        sumY += p.y;
        sumThetaX += cos(p.theta);
        sumThetaY += sin(p.theta);
    }

    // smoothing the state estimate using running average
    currentEstimate.x = currentEstimate.x * GAMMA + sumX/numParticles*(1 - GAMMA);
    currentEstimate.y = currentEstimate.y * GAMMA + sumY/numParticles*(1 - GAMMA);
    currentEstimate.theta = currentEstimate.theta * GAMMA + atan2(sumThetaY/numParticles, sumThetaX/numParticles)*(1 - GAMMA);
}


void resampleParticles(std::vector<RobotState>& particles, std::vector<double>& weights, double maxWeight)
{
    /*
        Resample the particles based on their weight using 'Resampling Wheel' method
        Moreover, add some random particles to encourage exploration
    */
    int N = particles.size();    
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
        // add some noise to the resampled particles to encourage diversity
        p.x = particles[index].x + gaussianRandom(0.0, RESAMPLING_NOISE);
        p.y = particles[index].y + gaussianRandom(0.0, RESAMPLING_NOISE);
        p.theta = particles[index].theta + gaussianRandom(0.0, RESAMPLING_NOISE*10);
        newParticles.push_back(p);
    }
    particles = newParticles;
}

void sampleRandomParticles(std::vector<RobotState>& particles, int N)
{
    for (int i = 0; i < N; ++i) {
        RobotState p;
        p.x = uniformCandidateDist(randEngine) * (METERS_PER_PIXEL * (FIELD_LENGTH/2 + FIELD_OFFSET_X));
        p.y = uniformCandidateDist(randEngine) * (METERS_PER_PIXEL * (FIELD_WIDTH/2 + FIELD_OFFSET_Y));
        p.theta = uniformCandidateDist(randEngine) * M_PI;
        particles[i] = p;
    }
}

void sampleCircular(std::vector<MarkerObservation>& observations, std::vector<RobotState>& particles, 
                     std::vector<FieldLocation>& landmarks, RobotParams& rParams, int N)
{
    /*
        Given current observations, based on the landmark's index we can sample candidates 
        around the landmark that satisfies the observation (i.e.: circle with radius = distance).
    */
    
    if (observations.empty())
        return;

    std::cout << "Circular Sampling of Particles ..." << std::endl;
    int nObs = observations.size();     // number of observed landmarks
    int candidatesPerLandmark = N/nObs;
    int j = 0;  // particle counter
    for (auto& obs : observations)
    {
        double landmarkX = landmarks[obs.markerIndex].x;
        double landmarkY = landmarks[obs.markerIndex].y;
        
        // sample candidates around the landmark 
        // (for each x sampled we have two possible y values)
        for (int i = 0; i < candidatesPerLandmark/2; i++)
        {
            double xLimitHigh = landmarkX + obs.distance;
            double xLimitLow = landmarkX - obs.distance;
            
            // check if x is in the valid range
            if (xLimitHigh > METERS_PER_PIXEL * (FIELD_LENGTH/2 + FIELD_OFFSET_X))
            {
                xLimitHigh = METERS_PER_PIXEL * (FIELD_LENGTH/2 + FIELD_OFFSET_X);
            }
            if (xLimitLow < -METERS_PER_PIXEL * (FIELD_LENGTH/2 + FIELD_OFFSET_X))
            {
                xLimitLow = -METERS_PER_PIXEL * (FIELD_LENGTH/2 + FIELD_OFFSET_X);
            }

            // Sampling x uniformly in the valid range
            double x = (uniformCandidateDist(randEngine) + 1)*(xLimitHigh - xLimitLow)/2 + xLimitLow;

            double y1 = landmarkY + sqrt(pow(obs.distance, 2) - pow(x - landmarkX, 2));
            double y2 = landmarkY - sqrt(pow(obs.distance, 2) - pow(x - landmarkX, 2));
            
            double yLimit = METERS_PER_PIXEL * (FIELD_WIDTH/2 + FIELD_OFFSET_Y);
            
            // check if ys are in the valid range
            if (y1 < yLimit && y1 > -yLimit)
            {
                y1 += gaussianRandom(0.0, rParams.sensor_noise_distance);
                particles[j] = {x, y1, uniformCandidateDist(randEngine) * M_PI};
                ++j;
            }
            
            if (y2 < yLimit && y2 > -yLimit)
            {
                y2 += gaussianRandom(0.0, rParams.sensor_noise_distance);
                particles[j] = {x, y2, uniformCandidateDist(randEngine) * M_PI};
                ++j;
            }
        }
    }
}