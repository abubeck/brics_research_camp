/*
 * File:   ArmMsg.h
 * Author: Özgür Sen
 *
 * Created on 2. Juni 2010, 12:43
 */

#ifndef _YOUBOT_SLAVE_MESSAGE_H
#define	_YOUBOT_SLAVE_MESSAGE_H

#include <ethercattype.h>
#include <string>

namespace soem_ethercat_drivers
{

    class YouBotSlaveMsg
    {

    public:
//    struct outputbuffer; master view
        struct outputBuffer{
//			int16   currentMotorPwm;
//			uint8   bldcControllerMode;
//			uint8   encoderMode;
//			uint16  maxCurrent;
//			int16   commutationOffsetCw;
//			int16   commutationOffsetCcw;
//			int16   compensationFactorSine;
//			int16   appCounter;
			int32  positionOrSpeed;
			uint8  controllerMode;
        } stctOutput;

//    struct inputbuffer; master view
        struct  inputBuffer{
			int32   actualPosition;
			int32   actualCurrent;
			int32   actualVelocity;
			uint16  errorFlags;
			uint16  driverTemperature;

//			uint16  statusFlags;
//			uint16  supplyVoltage;
//			int16   iPhaseA;
//			int16   iPhaseB;
//			int16   iPhaseC;
//			uint16  dutyCylePwmA;
//			uint16  dutyCylePwmB;
//			uint16  dutyCylePwmC;
//			uint16  rotorPhi;
//			uint16  encoderCommutationOffset;
//			uint16  softwareVersion;
//			uint16  appCounter;
	} stctInput;



        // Constructor
        YouBotSlaveMsg(){}

        // Copy-Constructor
        YouBotSlaveMsg(const YouBotSlaveMsg &copy) {
	    stctOutput = copy.stctOutput;
	    stctInput  = copy.stctInput;
        }

        // Destructor
        ~YouBotSlaveMsg(){}

        // assignment operator
        YouBotSlaveMsg & operator=(const YouBotSlaveMsg &copy)
        {
            if (this == &copy)
                return *this;

		    stctOutput = copy.stctOutput;
		    stctInput  = copy.stctInput;

            return *this;
        }
    };
}

#endif	/* _YOUBOT_SLAVE_MESSAGE_H */