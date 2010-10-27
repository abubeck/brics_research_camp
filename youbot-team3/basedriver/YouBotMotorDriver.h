// Author: Morten Kjaergaard morten.kjargaard@prevas.dk

#include <iostream>
#include <vector>
#include <cstdlib> // required by youBotApi.h
#include <youBotApi.h>

#define _DBG_MSGS

#ifdef _DBG_MSGS
#define DOUT(x) std::cout << x << std::endl
#else
define DOUT(x)
#endif

class YouBotMotorDriver {
private:
    bool mOpened;
    youbot::YouBotApi * mpApi;

    // todo: set parameters in some configuration method
    const static int cNumMotors = 4;
    const static float cEncPerRadian = 240.0f; // todo: Measured
    const static float cGearRatio = 25.84f;

    enum {
        eControllerModePosition = 1, eControllerModeSpeed = 2
    };

public:
    YouBotMotorDriver() :
        mOpened(false), mpApi(0) {
    }

    int Open(int key = 12345) {
        mOpened = true;
        return 0;

        mpApi = new youbot::YouBotApi("/tmp/youBotMemMapFile", key);

        for (int i = 0; i < 4; i++) {
            mpApi->setControllerMode(i, eControllerModeSpeed);
            mpApi->setMotorPositionOrSpeed(i, 0);
        }
        return 0;
    }

    /**
     * Set speed for all motors at once
     * @argumenst: radians per second
     */
    int SetSpeed(float aSpeed[], unsigned int aLength) {
        if (mOpened == false || aLength != cNumMotors) {
            return -1;
        }

        for (int i = 0; i < 4; i++) {
            mpApi->setControllerMode(i, eControllerModeSpeed);
            float AxisRpm = aSpeed[i] * (60.0f / (2*3.14f));
            int MotorSpeed = static_cast<int>(AxisRpm * cGearRatio);
            mpApi->setMotorPositionOrSpeed(i, EncSpeed);
            DOUT("SetSpeed Motor " << i << " Motor:" << MotorSpeed << " RadPerSec:" << aSpeed[i]);
        }
    }

    /**
     * Set speed for 4 motors at once
     * @argumenst: radians per second
     */
    int SetSpeed(float aSpeed1, float aSpeed2, float aSpeed3, float aSpeed4) {
        float Speed[4];
        Speed[0] = aSpeed1;
        Speed[1] = aSpeed2;
        Speed[2] = aSpeed3;
        Speed[3] = aSpeed4;
        SetSpeed(Speed, 4);
    }

    int SetPosition(float aRelPosition[]) {
        return -1;
    }

};

