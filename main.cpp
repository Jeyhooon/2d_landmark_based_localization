#include <stdio.h>
#include <stdlib.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include "robot_defs.h"
#include "controller.h"
#include "main.h"

// Defining global variables
RobotParams rParams;    // Robot model parameters
RobotState currentEstimate;   // Current robot position estimate
std::vector<FieldLocation> landmarks(NUM_LANDMARKS);    // Global location of each landmark
std::vector<RobotState> particles(MAX_NUM_PARTICLES);   // Particles for MCL algorithm
std::vector<double> weights(MAX_NUM_PARTICLES);         // Importance weights for each particle
bool isPVisualization = true;   // Flag to toggle particle visualization

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
    /*
        first we need to map the change in the local reference-frame into the global reference-frame and then add it to each particle
        (delta is already noisy, don't need to add noise to it)
    */ 

    for (auto& p : particles) 
    {
        p.x += delta.x * cos(p.theta) - delta.y * sin(p.theta);
        p.y += delta.x * sin(p.theta) + delta.y * cos(p.theta);
        p.theta += delta.theta;
        
        // to be sure theta stays in [-pi, pi] interval
        clipOrientation(p.theta);
    }

    // finally, updating the current estimate based on the updated particles
    updateCurrentEstimate(currentEstimate, particles);
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
    /*
        Using Monte Carlo Localization (MCL) algorithm to estimate the robot's state using particles
    */
    // Done TODO: Write your sensor update procedures here
    if (observations.empty() || particles.empty())
        return;

    // calculate particle importance weights (normalized) + maxWeight and number of effective particles (Neff)
    double maxWeight = -1;
    double Neff = 0;
    updateWeights(weights, particles, observations, landmarks, rParams, Neff, maxWeight);
    double probState = obsProbability(currentEstimate, observations, landmarks, rParams);
    std::cout << "probState: " << probState << std::endl;

    //  if effective number of particles is too low, resample (lack of diversity)
    if(Neff < RESAMPLING_THRESHOLD * static_cast<double>(particles.size()))
    {
        resampleParticles(particles, weights, maxWeight);
        if (probState < KIDNAPPED_THRESHOLD)
            sampleCircular(observations, particles, landmarks, rParams, static_cast<int>(particles.size() * MIN_RAND));
            
        // Clear weights as they're invalidated after resampling
        weights.clear();

        // finally, updating the current estimate based on the updated particles
        updateCurrentEstimate(currentEstimate, particles);
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
    // Done TODO: Write your initialization procedures here
    rParams = robotParams;
    currentEstimate = robotState;

    // pre-allocate memory
    landmarks.reserve(NUM_LANDMARKS);
    particles.reserve(MAX_NUM_PARTICLES);
    weights.reserve(MAX_NUM_PARTICLES);

    // store landmarks
    for (int i = 0; i < NUM_LANDMARKS; i++) {
        landmarks[i] = markerLocations[i];
    }

    if (INIT_DIST_TYPE == "uniform")
    {
        // initialize particles uniformly distributed on the field
        sampleRandomParticles(particles, MAX_NUM_PARTICLES);
    }
    else if (INIT_DIST_TYPE == "gaussian")
    {
        // initialize particles using Gaussian distribution around initial position (instead of uniform distribution on the field)
        // (e.g.: using the same sensor noise values for initial robot state)
        for (int i = 0; i < MAX_NUM_PARTICLES; i++) {
            RobotState p;
            p.x = robotState.x + gaussianRandom(0.0, rParams.sensor_noise_distance);
            p.y = robotState.y + gaussianRandom(0.0, rParams.sensor_noise_distance);
            p.theta = robotState.theta + gaussianRandom(0.0, rParams.sensor_noise_orientation);

            // Ensure theta stays in [-pi, pi] interval
            clipOrientation(p.theta);
            particles[i] = p;
        }
    }
    else
    {
        std::cout << "Invalid INIT_DIST_TYPE, using Uniform by default" << std::endl;
        sampleRandomParticles(particles, MAX_NUM_PARTICLES);
    }

    updateCurrentEstimate(currentEstimate, particles);
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
    
    // Draws Particles as cyan colored points at specified global locations on field
    // Draws red orientation line for each particle

    if(isPVisualization)
    {
        // Draw the particles
        int pixelX, pixelY;
        glPointSize(6.0);
        glBegin(GL_POINTS);
        glColor3f(0.0, 1.0, 1.0);
        for(const auto& p : particles){
            global2pixel(p.x, p.y, pixelX, pixelY);
            glVertex2i(pixelX, pixelY);
        }
        glEnd();

        // Draw orientation lines for each particle
        int endPixelX, endPixelY;
        glColor3f(1.0, 0.0, 0.0);  // Red color for orientation lines
        glBegin(GL_LINES);
        for(const auto& p : particles)
        {
            double endX = p.x + 4 * METERS_PER_PIXEL * cos(p.theta);
            double endY = p.y + 4 * METERS_PER_PIXEL * sin(p.theta);

            global2pixel(p.x, p.y, pixelX, pixelY); // start point
            global2pixel(endX, endY, endPixelX, endPixelY); // end point

            glVertex2i(pixelX, pixelY);
            glVertex2i(endPixelX, endPixelY);
        }
        glEnd();
    }

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
    // Done TODO: (Optional) Write your keyboard input handling procedures here

    // by pressing 'p' key, we can toggle the particle visualization
    if (key == 'p')
    {
        isPVisualization = !isPVisualization;
        return 1;
    }
	
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

