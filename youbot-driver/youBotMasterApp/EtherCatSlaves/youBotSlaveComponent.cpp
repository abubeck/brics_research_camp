
#include <youBotSlaveComponent.h>
#include <soem_driver_factory.h>
//#include <bitset>
#include <stdio.h>

namespace soem_ethercat_drivers
{

	YouBotSlaveComponent::YouBotSlaveComponent(ec_slavet* mem_loc): SoemDriver(mem_loc)	{
	}

	bool YouBotSlaveComponent::update(YouBotSlaveMsg& data) {
	//assign data to the Outputbuffer
	((outputBuffer*)(datap->outputs))->positionOrVelocity    = data.stctOutput.positionOrSpeed;
	((outputBuffer*)(datap->outputs))->controllerMode		 = data.stctOutput.controllerMode;
//		((outputBuffer*)(datap->outputs))->bldcControllerMode	 = data.stctOutput.bldcControllerMode;
//		((outputBuffer*)(datap->outputs))->bldcControllerMode	 = data.stctOutput.bldcControllerMode;
//		((outputBuffer*)(datap->outputs))->currentMotorPwm		 = data.stctOutput.currentMotorPwm;
//		((outputBuffer*)(datap->outputs))->bldcControllerMode	 = data.stctOutput.bldcControllerMode;
//		((outputBuffer*)(datap->outputs))->encoderMode		     = data.stctOutput.encoderMode;
//		((outputBuffer*)(datap->outputs))->maxCurrent			 = data.stctOutput.maxCurrent;
//		((outputBuffer*)(datap->outputs))->commutationOffsetCw   = data.stctOutput.commutationOffsetCw;
//		((outputBuffer*)(datap->outputs))->commutationOffsetCww  = data.stctOutput.commutationOffsetCcw;
//		((outputBuffer*)(datap->outputs))->compensationFactor	 = data.stctOutput.compensationFactorSine;
//		((outputBuffer*)(datap->outputs))->appCounter			 = data.stctOutput.appCounter;



		//get values from Inputbuffer and assign them to the data object
		data.stctInput.actualPosition			= ((inputBuffer*)(datap->inputs))->actualPosition;
	    data.stctInput.actualCurrent			= ((inputBuffer*)(datap->inputs))->actualCurrent;
		data.stctInput.actualVelocity			= ((inputBuffer*)(datap->inputs))->actualVelocity;
		data.stctInput.errorFlags				= ((inputBuffer*)(datap->inputs))->errorFlags;
		data.stctInput.driverTemperature		= ((inputBuffer*)(datap->inputs))->temp;

//		data.stctInput.statusFlags				= ((inputBuffer*)(datap->inputs))->statusFlags;
//	    data.stctInput.supplyVoltage			= ((inputBuffer*)(datap->inputs))->vcc;
//		data.stctInput.iPhaseA					= ((inputBuffer*)(datap->inputs))->iPhaseA;
//		data.stctInput.iPhaseB					= ((inputBuffer*)(datap->inputs))->iPhaseB;
//		data.stctInput.iPhaseC					= ((inputBuffer*)(datap->inputs))->iPhaseC;
//		data.stctInput.dutyCylePwmA				= ((inputBuffer*)(datap->inputs))->dutyCylePwmA;
//		data.stctInput.dutyCylePwmB				= ((inputBuffer*)(datap->inputs))->dutyCylePwmB;
//		data.stctInput.dutyCylePwmC				= ((inputBuffer*)(datap->inputs))->dutyCylePwmC;
//	    data.stctInput.rotorPhi                 = ((inputBuffer*)(datap->inputs))->rotorPhi;
//	    data.stctInput.encoderCommutationOffset	= ((inputBuffer*)(datap->inputs))->encCommutOffset;
//	    data.stctInput.softwareVersion			= ((inputBuffer*)(datap->inputs))->swVersion;
//	    data.stctInput.appCounter               = ((inputBuffer*)(datap->inputs))->appCounter;

			

		return true;
	}

	namespace {
		SoemDriver* createYouBotSlaveComponent(ec_slavet* mem_loc) {
		  return new YouBotSlaveComponent(mem_loc);
		}

		const bool registered = SoemDriverFactory::Instance().registerDriver("TMCM-174",createYouBotSlaveComponent);
	}
}