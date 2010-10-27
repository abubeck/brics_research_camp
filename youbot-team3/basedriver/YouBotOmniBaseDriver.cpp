#include "YouBotOmniBaseDriver.h"

YouBotOmniBaseDriver::YouBotOmniBaseDriver()
{
}

int YouBotOmniBaseDriver::Open(int aKey)
{
    mMotorDriver.Open(aKey);
}

int YouBotOmniBaseDriver::SetVelocity(float aX_vel, float aY_vel, float aTheta_vel)
{
    std::cout << "SetVelocity" << std::endl;

    // Requred wheen rotation for X and Y component
    float RadPerSec_FromX = aX_vel / (cWheelDiam / 2);
    float RadPerSec_FromY = aY_vel / (cWheelDiam * cSlideRatio / 2);

    // Calculate Rotation Component
    float Omkreds = sqrt(cWheelDist * cWheelDist + cAxisWidth * cAxisWidth) * cPI;
    float RequestedMovement = Omkreds * (aTheta_vel / (2 * cPI));
    float OneWheelRotation = cRotationRatio * cWheelDiam * cPI;
    float RadPerSec_FromTheta = 2 * cPI * (RequestedMovement / OneWheelRotation);

    // Calcuate required rad/sec for each X/Y component
    float M1 = -RadPerSec_FromX + RadPerSec_FromY + RadPerSec_FromTheta;
    float M2 =  RadPerSec_FromX + RadPerSec_FromY + RadPerSec_FromTheta;
    float M3 = -RadPerSec_FromX - RadPerSec_FromY + RadPerSec_FromTheta;
    float M4 =  RadPerSec_FromX - RadPerSec_FromY + RadPerSec_FromTheta;

    return mMotorDriver.SetSpeed(M1, M2, M3, M4);
}
