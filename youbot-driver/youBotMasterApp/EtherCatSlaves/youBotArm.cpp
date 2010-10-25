
#include <youBotArm.h>
#include <soem_driver_factory.h>
//#include <bitset>
#include <stdio.h>

#include <iostream>
using namespace std;

namespace soem_ethercat_drivers
{

	YouBotArm::YouBotArm(ec_slavet* mem_loc): SoemDriver(mem_loc){
	}

	bool YouBotArm::update(YouBotArmMsg& data) 
	{
	//assign data to the Outputbuffer
//cout << "LINE:" << __LINE__ << endl;
	((outputBuffer*)(datap->outputs))->positionOrVelocity    = data.stctOutput.positionOrSpeed;
	((outputBuffer*)(datap->outputs))->controllerMode		 = data.stctOutput.controllerMode;

	//get values from Inputbuffer and assign them to the data object
	data.stctInput.actualPosition			= ((inputBuffer*)(datap->inputs))->actualPosition;
	data.stctInput.actualCurrent			= ((inputBuffer*)(datap->inputs))->actualCurrent;
	data.stctInput.actualVelocity			= ((inputBuffer*)(datap->inputs))->actualVelocity;
	data.stctInput.errorFlags				= ((inputBuffer*)(datap->inputs))->errorFlags;
	data.stctInput.driverTemperature		= ((inputBuffer*)(datap->inputs))->temp;

	return true;
	}

	namespace 
	{
		SoemDriver* createYouBotArm(ec_slavet* mem_loc) 
		{
		  return new YouBotArm(mem_loc);
		}
		
		const bool registered = SoemDriverFactory::Instance().registerDriver("TMCM-KR-841",createYouBotArm);
	}
}
