#include "YouBotMotorDriver.h"
#include <cmath>

class YouBotOmniBaseDriver
{
private:
    YouBotMotorDriver mMotorDriver;

    // todo: set parameters in some configuration method
    const static float cWheelDiam = 0.07f; // [m] (todo: approx guess, measure)
    const static float cSlideRatio = 0.95f; // how far sideways in one wheel rotation compared to forward (todo: approx guess, calibrate?)
    const static float cRotationRatio = 1.0f; // todo: calculate from geometry
    const static float cWheelDist = 0.46f; // [m]
    const static float cAxisWidth = 0.3f; // [m]
    const static float cPI = 3.14f; // todo: pull this from some library

public:
    YouBotOmniBaseDriver();

    int Open( int aKey = 12345 );

    /**
     * X&Y is m/s
     * Theta is rad per second
     *
     * currently assumes motors are located:
     * 1 ----- 2
     *   |   |
     *   |   |
     *   |   |
     * 3 ----- 4
     */
    int SetVelocity( float aX_vel, float aY_vel, float aTheta_vel );
};
