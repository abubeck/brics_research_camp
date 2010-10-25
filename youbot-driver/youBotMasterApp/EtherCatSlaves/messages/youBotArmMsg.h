/*
 * File:   ArmMsg.h
 * Author: Özgür Sen
 *
 * Created on 20. October 2010, 02:22
 */

#ifndef _YOUBOT_ARM_MESSAGE_H
#define	_YOUBOT_ARM_MESSAGE_H

#include <ethercattype.h>
#include <string>

namespace soem_ethercat_drivers
{

    class YouBotArmMsg
    {

    public:
//    struct outputbuffer; master view
        struct outputBuffer{
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
	} stctInput;



        // Constructor
        YouBotArmMsg(){}

        // Copy-Constructor
        YouBotArmMsg(const YouBotArmMsg &copy) {
	    stctOutput = copy.stctOutput;
	    stctInput  = copy.stctInput;
        }

        // Destructor
        ~YouBotArmMsg(){}

        // assignment operator
        YouBotArmMsg & operator=(const YouBotArmMsg &copy)
        {
            if (this == &copy)
                return *this;

		    stctOutput = copy.stctOutput;
		    stctInput  = copy.stctInput;

            return *this;
        }
    };
}

#endif	/* _YOUBOT_ARM_MESSAGE_H */
