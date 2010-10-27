/*
 * File:   youBotApi.h
 * Author: Özgür Sen
 *
 * Created on 13. September 2010
 */

#ifndef _YOUBOT_API_H
#define	_YOUBOT_API_H

#include <youBotSlaveMsg.h>
#include <MemoryMappedFiles.h>
#include <SemaphoreLock.h>

#include <stdio.h>

//using namespace memmap;
//using namespace semlock;

namespace youbot {

typedef struct{
	int jointID;
	int gearRatio;
	double maxJointValue;
	//double maxAxisValue;
	double minJointValue;
	//double minAxisValue;
	bool negative;
	double rpm_to_rad;
} YouJoint;

//! API for controlling the youBot.
//! This API connects to the youBot-daemon,
//! So make sure that the daemon is running.

	class YouBotApi {
	   int encoderSteps;

	   YouJoint joints[5];

	public:

	   // Convert from encoder increment to joint space angle (relative) in radians
	   double encoderValueToJointValue(int jointID, int encoderValue){
	       return double(encoderValue)/(joints[jointID-1].gearRatio*encoderSteps)*(2*M_PI);
	   }
	   // Convert from joint angle (relative) in radians to encoder increment value
	   int jointValueToEncoderValue(int jointID, double jointValue){
	       return jointValue/(2*M_PI)*(joints[jointID-1].gearRatio*encoderSteps);
	   }

	   //Convert from encoder speeds to joint speeds in radians per second
	   double encoderSpeedToRadPerSec(int jointID, double encoderSpeed){
		   return encoderSpeed*joints[jointID-1].rpm_to_rad;
	   }

	   //Convert from joint speeds in radians per second to encoder speeds
	   double radPerSecToEncoderSpeed(int jointID, double radPerSec){
	   	   return radPerSec/joints[jointID-1].rpm_to_rad;
	   }

	   // Convert from Joint value to axis absolute value
	   int getAxisAbsolutePosition(int jointID, double jointPosition){
	     if(joints[jointID-1].negative)
	         return jointValueToEncoderValue(jointID, -jointPosition + joints[jointID-1].minJointValue);
	       else
	         return jointValueToEncoderValue(jointID, jointPosition - joints[jointID-1].maxJointValue);
	   }

	   // Convert from Joint value to axis absolute value
	   double getJointAbsolutePosition(int jointID, int axisPosition){
	       double value = encoderValueToJointValue(jointID, axisPosition);
	       if(joints[jointID-1].negative)
	         return joints[jointID-1].minJointValue - value;
	       else
	         return value + joints[jointID-1].maxJointValue;
	   }

		//ctor
		YouBotApi(const char * file, int key ) :
		          encoderSteps(4096) {
			
			int ec = memMap.openMappedFile(file);
			if(ec != 0){
				printf("MapFile couldn't be opened. Check if you have permissions to read/write the file: %s", file);
				exit(0);
			}

			mappedMsg = (soem_ethercat_drivers::YouBotSlaveMsg *)memMap.getAddr();
			semLock.openLock(key);

	    	// Joint 1 parameters
	    	joints[0].jointID = 1;
	    	joints[0].gearRatio = 156;
	    	joints[0].minJointValue = -169./180.*M_PI;
	    	joints[0].maxJointValue = 169./180.*M_PI;
	    	joints[0].negative = true;
	    	joints[0].rpm_to_rad = -0.0783;

	    	// Joint 2 parameters
	    	joints[1].jointID = 2;
	    	joints[1].gearRatio = 156;
	    	joints[1].minJointValue = -65./180.*M_PI;
	    	joints[1].maxJointValue = 90./180.*M_PI;
	    	joints[1].negative = true;
	    	joints[1].rpm_to_rad = -0.0807;

	    	// Joint 3 parameters
	    	joints[2].jointID = 3;
	    	joints[2].gearRatio = 100;
	    	joints[2].minJointValue = -151./180.*M_PI;
	    	joints[2].maxJointValue = 146./180.*M_PI;
	    	joints[2].negative = false;
	    	joints[2].rpm_to_rad = 0.0863;

	    	// Joint 4 parameters
	    	joints[3].jointID = 4;
	    	joints[3].gearRatio = 71;
	    	joints[3].minJointValue = -102.5/180.*M_PI;
	    	joints[3].maxJointValue = 102.5/180.*M_PI;
	    	joints[3].negative = true;
	    	joints[3].rpm_to_rad = -0.0863;

	    	// Joint 5 parameters
	    	joints[4].jointID = 5;
	    	joints[4].gearRatio = 71;
	    	joints[4].minJointValue = -167.5/180.*M_PI;
	    	joints[4].maxJointValue = 167.5/180.*M_PI;
	    	joints[4].negative = true;
	    	joints[4].rpm_to_rad = -0.0825;
		}

		//dtor
		~YouBotApi() {
			mappedMsg = NULL;
			memMap.unmapFile();
			semLock.unlock();
		}

	//Setter-Methods/*		//! Set the motor PWM value for ta certain he slave component
		//! input:
		//!		slaveNr:  The nr of the slave-component, whose value hast to be set
		//!		pwmValue: The value of the PWM
		//!	return:
		//!		errorCode: 0  OK
		//!		errorCode: -1 error
		int setMotorPositionOrSpeed(int slaveNr, int value) {
			semLock.lock();
			mappedMsg[slaveNr].stctOutput.positionOrSpeed = value;
			semLock.unlock();
			return 0;
		}

		//! Set the BLDC controller values for a certain slave component
		//! input:
		//!		slaveNr:  The nr of the slave-component, whose value hast to be set
		//!		ctrlMode: The value of the BLDC controller mode
		//!	return:
		//!		errorCode: 0  OK
		//!		errorCode: -1 error
		int setControllerMode(int slaveNr, int ctrlMode) {
			semLock.lock();
			mappedMsg[slaveNr].stctOutput.controllerMode = ctrlMode;
			semLock.unlock();
			return 0;
		}
		
		/**
		 * Sets the base velocity (m/s, rad/s)
		 */
		int setBaseVelocity(double forward, double right, double yaw) {
			for(int i=0; i<4; i++) setControllerMode(i, 2);
			double gearbox = 9405.0 / 364.0;
			double forwardTicks = forward * 330000 / gearbox;
			double rightTicks = right * 346000 / gearbox;
			double yawTicks = yaw / 2 / 3.1415926 * 805000 / gearbox;

			setMotorPositionOrSpeed(0, (int)(-forwardTicks - rightTicks + yawTicks));
			setMotorPositionOrSpeed(1, (int)(forwardTicks - rightTicks + yawTicks));
			setMotorPositionOrSpeed(2, (int)(-forwardTicks + rightTicks + yawTicks));
			setMotorPositionOrSpeed(3, (int)(forwardTicks + rightTicks + yawTicks));

		}

		//! Set the Position for the Axis
		//! input:
		//!		axisNr:  The nr of the axis, whose value hast to be set
		//!		pos:     The value of the position, to be set
		//!	return:
		//!		errorCode: 0  OK
		//!		errorCode: -1 error
		int setAxisPosition(int axisNr, int pos) {
			semLock.lock();
			axisNr--;
			switch( axisNr ) 
			{ 	
				case  0:	
					if(pos < -587000)	pos = -587000; 
					break; 
				case  1:	
					if(pos < -269000)	pos = -269000; 
					break; 
				case  2:	
					if(pos < -325000)	pos = -325000; 
					break; 
				case  3:
					if(pos < -162000)	pos = -162000; 
					break;
				case  4:
					if(pos < -264990)	pos = -264990; 
					break;
			}
			
			if(pos > 0) pos = 0;
			
			mappedMsg[axisNr+4].stctOutput.positionOrSpeed = pos;
			semLock.unlock();
			return 0;
		}
		
		//! To open or close the gripper
		//! input:
		//!		action: 0 = do nothing; 1 = open ; 2 = close;
		//!	return:
		//!		errorCode: 0  OK
		//!		errorCode: -1 error
		int setGripper(int action) {
			semLock.lock();
			mappedMsg[9].stctOutput.positionOrSpeed = action;
			mappedMsg[9].stctOutput.controllerMode = 1;
			semLock.unlock();
			return 0;
		}
		
/*		//! Set the motor PWM value for ta certain he slave component
		//! input:
		//!		slaveNr:  The nr of the slave-component, whose value hast to be set
		//!		pwmValue: The value of the PWM
		//!	return:
		//!		errorCode: 0  OK
		//!		errorCode: -1 error */
// 		int setCurrentMotorPwm(int slaveNr, int pwmValue) {
// 			semLock.lock();
// 			mappedMsg[slaveNr].stctOutput.currentMotorPwm = pwmValue;
// 			semLock.unlock();
// 			return 0;
// 		}
// 
// 		//! Set the BLDC controller values for a certain slave component
// 		//! input:
// 		//!		slaveNr:  The nr of the slave-component, whose value hast to be set
// 		//!		ctrlMode: The value of the BLDC controller mode
// 		//!	return:
// 		//!		errorCode: 0  OK
// 		//!		errorCode: -1 error
// 		int setBldcControllerMode(int slaveNr, int ctrlMode) {
// 			semLock.lock();
// 			mappedMsg[slaveNr].stctOutput.bldcControllerMode = ctrlMode;
// 			semLock.unlock();
// 			return 0;
// 		}
// 
// 		//! Set the max current for a certain slave component
// 		//! input:
// 		//!		slaveNr:    The nr of the slave-component, whose value hast to be set
// 		//!		maxCurrent: The maximum value of the current
// 		//!	return:
// 		//!		errorCode: 0  OK
// 		//!		errorCode: -1 error
// 		int setMaxCurrent(int slaveNr, int maxCurrent) {
// 			semLock.lock();
// 			mappedMsg[slaveNr].stctOutput.maxCurrent = maxCurrent;
// 			semLock.unlock();
// 			return 0;
// 		}
// 
// 		//! Other setter-Methods according to the youBot protocoll specification
// 		int setAppCounter(int slaveNr, int appCounter){
// 			semLock.lock();
// 			mappedMsg[slaveNr].stctOutput.appCounter = appCounter;
// 			semLock.unlock();
// 			return 0;
// 		}
// 
// 		int setEncoderMode(int slaveNr, int encoderMode){
// 			semLock.lock();
// 			mappedMsg[slaveNr].stctOutput.encoderMode = encoderMode;
// 			semLock.unlock();
// 			return 0;
// 		}
// 
// 		int setCompensationFactor(int slaveNr, int compensationFactor){
// 			semLock.lock();
// 			mappedMsg[slaveNr].stctOutput.compensationFactorSine = compensationFactor;
// 			semLock.unlock();
// 			return 0;
// 		}
// 
// 		int setCommutationOffsetCw(int slaveNr, int cwValue){
// 			semLock.lock();
// 			mappedMsg[slaveNr].stctOutput.commutationOffsetCw = cwValue;
// 			semLock.unlock();
// 			return 0;
// 		}
// 
// 		int setCommutationOffsetCww(int slaveNr, int cwwValue){
// 			semLock.lock();
// 			mappedMsg[slaveNr].stctOutput.commutationOffsetCcw = cwwValue;
// 			semLock.unlock();
// 			return 0;
// 		}

	//Getter-Methods
		//! Get the input values according to the youBot protocoll specification
		//!	input:
		//!		slaveNr: The nr of the slave-component, whose value should be returned
		//!	return:
		//!		The value of a certain input-parameter
		int32	getActualPosition(int slaveNr) { return mappedMsg[slaveNr].stctInput.actualPosition; }
		int32	getActualCurrent(int slaveNr) { return mappedMsg[slaveNr].stctInput.actualCurrent; }
		int32	getActualVelocity(int slaveNr) { return mappedMsg[slaveNr].stctInput.actualVelocity; }
		uint16	getErrorFlags(int slaveNr) { return mappedMsg[slaveNr].stctInput.errorFlags; }
 		uint16	getTemparature(int slaveNr) { return mappedMsg[slaveNr].stctInput.driverTemperature; }
		int32	getAxisPosition(int axisNr) { return mappedMsg[axisNr+3].stctInput.actualPosition; }
		
//		uint16	getStatusFlags(int slaveNr) { return mappedMsg[slaveNr].stctInput.statusFlags; }
// 		uint16	getVcc(int slaveNr) { return mappedMsg[slaveNr].stctInput.supplyVoltage; }
// 		int16	getIPhaseA(int slaveNr) { return mappedMsg[slaveNr].stctInput.iPhaseA; }
// 		int16	getIPhaseB(int slaveNr) { return mappedMsg[slaveNr].stctInput.iPhaseB; }
// 		int16	getIPhaseC(int slaveNr) { return mappedMsg[slaveNr].stctInput.dutyCylePwmC; }
// 		uint16	getRotorPhi(int slaveNr) { return mappedMsg[slaveNr].stctInput.rotorPhi; }
// 		uint16	getAppCounter(int slaveNr) { return mappedMsg[slaveNr].stctInput.errorFlags; }
// 		uint16	getErrorFlags(int slaveNr) { return mappedMsg[slaveNr].stctInput.statusFlags; }
// 		uint16	getStatusFlags(int slaveNr) { return mappedMsg[slaveNr].stctInput.statusFlags; }
// 		uint16	getTemparature(int slaveNr) { return mappedMsg[slaveNr].stctInput.driverTemperature; }
// 		uint16	getDutyCylePwmA(int slaveNr) { return mappedMsg[slaveNr].stctInput.dutyCylePwmA; }
// 		uint16	getDutyCylePwmB(int slaveNr) { return mappedMsg[slaveNr].stctInput.dutyCylePwmB; }
// 		uint16	getDutyCylePwmC(int slaveNr) { return mappedMsg[slaveNr].stctInput.dutyCylePwmC; }
// 		int16	getActualCurrent(int slaveNr) { return mappedMsg[slaveNr].stctInput.actualCurrent; }
// 		int32	getActualPosition(int slaveNr) { return mappedMsg[slaveNr].stctInput.actualPosition; }
// 		uint16  getSoftwareVersion(int slaveNr) { return mappedMsg[slaveNr].stctInput.softwareVersion; }
// 		uint16  getCommutationOffset(int slaveNr) { return mappedMsg[slaveNr].stctInput.encoderCommutationOffset; }



		private:

		//semaphore-objekt
		semlock::SemaphoreLock semLock;

		//member for a memory mapped file
		memmap::MemmoryMappedFiles memMap;

		//create a msg-object-pointer and assign the address of the mapped file
		//after that use the pointer like an array
		soem_ethercat_drivers::YouBotSlaveMsg * mappedMsg;
	};
}

#endif	/* _YOUBOT_API_H */
