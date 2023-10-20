#include <stdio.h>
#include <stdlib.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <math.h>
#include <vector>
#include <random>
#include "robot_defs.h"
#include "controller.h"
#include "main.h"

// Defining global variables
RobotState currentEstimate;   // Current robot position estimate
std::vector<FieldLocation> landmarks; // Global location of each landmark
std::vector<RobotState> particles;
static std::default_random_engine randEngine(std::random_device{}());
std::uniform_real_distribution<double> randRealDist(-1.0, 1.0);

// Helper functions
double pdfGaussian(double mu, double sigma, double x)
{
    // Probability of x for 1-dim Gaussian with mean mu and stddev sigma
    return exp(-(pow((mu - x), 2)) / (pow(sigma, 2)) / 2.0) / sqrt(2.0 * M_PI * (pow(sigma, 2)));
}

void updateCurrentEstimate()
{
    

    // only update the current estimate if there are particles
    if(!particles.empty())
    {
        // use mean values of particles as current estimate
        double sumX = 0, sumY = 0, sumTheta = 0;
        for (auto& p : particles) {
            sumX += p.x;
            sumY += p.y;
            sumTheta += p.theta;
        }

        double numParticles = static_cast<double>(particles.size());
        currentEstimate.x = sumX/numParticles;
        currentEstimate.y = sumY/numParticles;
        currentEstimate.theta = sumTheta/numParticles;
    }
}

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
    updateCurrentEstimate();
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
    currentEstimate = robotState;

    // pre-allocate memory
    landmarks.reserve(NUM_LANDMARKS);
    particles.reserve(MAX_NUM_PARTICLES);

    // store landmarks
    for (int i = 0; i < NUM_LANDMARKS; i++) {
        landmarks.push_back(markerLocations[i]);
    }

    // initialize particles uniformly distributed
    for (int i = 0; i < MAX_NUM_PARTICLES; i++) {
        RobotState particle;
        particle.x = randRealDist(randEngine) * (METERS_PER_PIXEL * (FIELD_LENGTH/2 + FIELD_OFFSET_X));
        particle.y = randRealDist(randEngine) * (METERS_PER_PIXEL * (FIELD_WIDTH/2 + FIELD_OFFSET_Y));
        particle.theta = randRealDist(randEngine) * M_PI;
        particles.push_back(particle);
    }

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

