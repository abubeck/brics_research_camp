/*
 * File:   youBotArm.h
 * Author: Özgür Sen
 *
 * Created on 20. October 2010
 */

#ifndef _YOUBOT_ARM_H
#define	_YOUBOT_ARM_H

#include "youBotArmMsg.h"
#include <soem_driver.h>

namespace soem_ethercat_drivers
{
    class YouBotArm : public SoemDriver
    {
        typedef struct PACKED
        {
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
        }inputBuffer;

    public:
        YouBotArm(ec_slavet* mem_loc);
        ~YouBotArm(){};

        bool update(YouBotArmMsg & data);
    };
}
#endif	/* _YOUBOT_ARM_H */
