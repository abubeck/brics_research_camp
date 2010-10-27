/*
 * File:   youBotSlaveComponent.h
 * Author: Özgür Sen
 *
 * Created on 13. September 2010
 */

#ifndef _YOUBOT_SLAVE_COMPONENT_H
#define	_YOUBOT_SLAVE_COMPONENT_H

#include "messages/youBotSlaveMsg.h"
#include <soem_driver.h>

namespace soem_ethercat_drivers
{
    class YouBotSlaveComponent : public SoemDriver
    {
        typedef struct PACKED
        {
//	    int16  currentMotorPwm;
//	    uint8  bldcControllerMode;
//		uint8  encoderMode;
//	    int32  sIntReserved[5];
//	    uint16 uIntReserved[2];
//	    uint16 maxCurrent;
//	    int16  commutationOffsetCw;
//		int16  commutationOffsetCww;
//	    int16  compensationFactor;
//	    int16  reservedValue;
//	    int16  appCounter;
		int32  positionOrVelocity;
		uint8  controllerMode;
        }outputBuffer;

	typedef struct PACKED
        {
	    int32   actualPosition;
		int32   actualCurrent;
		int32   actualVelocity;
		uint16  errorFlags;
	    uint16  temp;
//	    uint16  vcc;
//	    int16   iPhaseA;
//		int16   iPhaseB;
//		int16   iPhaseC;
//	    uint16  dutyCylePwmA;
//		uint16  dutyCylePwmB;
//		uint16  dutyCylePwmC;
//	    uint16  rotorPhi;
//	    uint16  encCommutOffset;
//	    uint16  uIntReserved[3];
//	    uint16  swVersion;
//	    uint16  appCounter;
        }inputBuffer;

    public:
        YouBotSlaveComponent(ec_slavet* mem_loc);
        ~YouBotSlaveComponent(){};

        bool update(YouBotSlaveMsg & data);
    };
}
#endif	/* _YOUBOT_SLAVE_COMPONENT_H */
