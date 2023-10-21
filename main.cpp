#include <stdio.h>
#include <stdlib.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include "robot_defs.h"
#include "controller.h"
#include "main.h"

// Defining global variables
RobotParams rParams; 
RobotState currentEstimate;   // Current robot position estimate
std::vector<FieldLocation> landmarks(NUM_LANDMARKS);  // Global location of each landmark
std::vector<RobotState> particles(MAX_NUM_PARTICLES); // Particles for MCL algorithm
std::vector<RobotState> newParticles(MAX_NUM_PARTICLES);
std::vector<double> weights(MAX_NUM_PARTICLES);       // Importance weights for each particle

/**
 * getRobotPositionEstimate()
 * This function is called by the controller to retrieve the current 
 * robot position estimate. 
 */
void getRobotPositionEstimate(RobotState& estimatePosn)
{
    // Done TODO: Write your procedures to set the current robot position estimate here
    estimatePosn = currentEstimate;
}

/**
 * motionUpdate()
 * This function is called every time the position of the robot is
 * updated. The argument passed is the relative change in position of the 
 * robot in local robot coordinates (observed by odometry model), which 
 * may be subject to noise (according to motion model parameters).
 */
void motionUpdate(RobotState delta)
{
    // Done TODO: Write your motion update procedures here
    // first we need to map the change in the local reference-frame into the global reference-frame and then add it to the current estimate
    double globalDeltaX = delta.x * cos(currentEstimate.theta) - delta.y * sin(currentEstimate.theta);
    double globalDeltaY = delta.x * sin(currentEstimate.theta) + delta.y * cos(currentEstimate.theta);

    // now we need to update the particles  
    // (motion noises might already be present in delta values so we don't need to add it here)
    for (auto& p : particles) {
        p.x += globalDeltaX;
        p.y += globalDeltaY;
        p.theta += delta.theta;
        
        // to be sure theta stays in [-pi, pi] interval
        if (p.theta > M_PI) p.theta -= 2 * M_PI;
        if (p.theta < -M_PI) p.theta += 2 * M_PI;
    }

    // finally, updating the current estimate based on the updated particles
    updateCurrentEstimate(currentEstimate, particles, weights);
}

/**
 * sensorUpdate()
 * This function is called every time the robot detects one or more
 * landmarks in its field of view. The argument passed contains all 
 * marker observations (marker index and position of marker in robot 
 * coordinates) for the current frame.
 */
void sensorUpdate(std::vector<MarkerObservation> observations)
{
    // TODO: Write your sensor update procedures here
    std::cout << "Landmark index: " << observations[0].markerIndex;
    std::cout <<" x: " << landmarks[observations[0].markerIndex].x <<  " y: " << landmarks[observations[0].markerIndex].y << std::endl;

    if (!observations.empty() && !particles.empty())
    {
        // calculate particle importance weights
        double maxWeight = -1;
        double Neff = 0;
        updateWeights(weights, particles, observations, landmarks, rParams, maxWeight, Neff);

        int N = particles.size();
        int N_resample = (1 - ALPHA) * N;
        int N_random = N - N_resample;

        //  if effective number of particles is too low, resample (lack of diversity)
        if(Neff < RESAMPLING_THRESHOLD * static_cast<double>(N))
        {
            std::cout << "Resampling..." << std::endl;

            //Resample the particles using Resampling Wheel technique
            newParticles.clear();
            std::uniform_real_distribution<double> randRealDist(0, 1.0);
            double beta = 0;
            int index = randRealDist(randEngine) * N;

            std::normal_distribution<double> noiseDist(0.0, 0.005);
            for (int i = 0; i < N_resample; i++)
            {
                beta += randRealDist(randEngine) * 2 * maxWeight;
                while(beta > weights[index])
                {
                    beta -= weights[index];
                    index = (index + 1) % N;
                }
                RobotState p;
                p.x = particles[index].x + noiseDist(randEngine);
                p.y = particles[index].y + noiseDist(randEngine);
                p.theta = particles[index].theta + noiseDist(randEngine);
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

            // recomputing weights

            
            // finally, updating the current estimate based on the updated particles
            updateCurrentEstimate(currentEstimate, particles, weights);
        }        

        
    }
    
}

/**
 * myinit()
 * Initialization function that takes as input the initial 
 * robot state (position and orientation), and the locations
 * of each landmark (global x,y coordinates).
 */
void myinit(RobotState robotState, RobotParams robotParams, 
            FieldLocation markerLocations[NUM_LANDMARKS])
{
    // TODO: Write your initialization procedures here
    rParams = robotParams;
    currentEstimate = robotState;

    // pre-allocate memory
    landmarks.reserve(NUM_LANDMARKS);
    particles.reserve(MAX_NUM_PARTICLES);
    newParticles.reserve(MAX_NUM_PARTICLES);
    weights.reserve(MAX_NUM_PARTICLES);

    // store landmarks
    for (int i = 0; i < NUM_LANDMARKS; i++) {
        landmarks[i] = markerLocations[i];
    }

    // initialize particles uniformly distributed on the field
    std::uniform_real_distribution<double> randRealDist(-1.0, 1.0);

    for (int i = 0; i < MAX_NUM_PARTICLES; i++) {
        RobotState p;
        p.x = randRealDist(randEngine) * (METERS_PER_PIXEL * (FIELD_LENGTH/2 + FIELD_OFFSET_X));
        p.y = randRealDist(randEngine) * (METERS_PER_PIXEL * (FIELD_WIDTH/2 + FIELD_OFFSET_Y));
        p.theta = randRealDist(randEngine) * M_PI;
        particles[i] = p;
    }

    // initialize particles using Gaussian distribution around initial position (instead of uniform distribution on the field)
    // (using the same sensor noise values for initial robot state)
    // std::normal_distribution<double> gaussX(robotState.x, rParams.sensor_noise_distance);
    // std::normal_distribution<double> gaussY(robotState.y, rParams.sensor_noise_distance);
    // std::normal_distribution<double> gaussTheta(robotState.theta, rParams.sensor_noise_orientation);

    // for (int i = 0; i < MAX_NUM_PARTICLES; i++) {
    //     RobotState p;
    //     p.x = gaussX(randEngine);
    //     p.y = gaussY(randEngine);
    //     p.theta = gaussTheta(randEngine);

    //     // Ensure theta stays in [-pi, pi] interval
    //     if (p.theta > M_PI) p.theta -= 2 * M_PI;
    //     if (p.theta < -M_PI) p.theta += 2 * M_PI;
        
    //     particles[i] = p;
    // }

    updateCurrentEstimate(currentEstimate, particles, weights);

}

/**
 * mydisplay()
 * This function is called whenever the display is updated. The controller
 * will draw the estimated robot position after this function returns.
 */
void mydisplay()
{
    // Done TODO: Write your drawing procedures here 
    //       (e.g., robot position uncertainty representation)
    
    // Draw Particles as cyan colored points at specified global locations on field
    // TODO: maybe draw arrows instead of points

    int pixelX, pixelY;
    glPointSize(5.0);
    glBegin(GL_POINTS);
    glColor3f(0.0, 1.0, 1.0);
    for(const auto& p : particles){
        global2pixel(p.x, p.y, pixelX, pixelY);
        glVertex2i(pixelX, pixelY);
    }
    glEnd();

}

/**
 * mykeyboard()
 * This function is called whenever a keyboard key is pressed, after
 * the controller has processed the input. It receives the ASCII value 
 * of the key that was pressed.
 *
 * Return value: 1 if window re-draw requested, 0 otherwise
 */
int mykeyboard(unsigned char key)
{
    // TODO: (Optional) Write your keyboard input handling procedures here
	
	return 0;
}


/**
 * Main entrypoint for the program.
 */
int main (int argc, char ** argv)
{
    // Initialize world, sets initial robot position
    // calls myinit() before returning
    runMainLoop(argc, argv);

    return 0;
}

