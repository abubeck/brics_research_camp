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
#include <math.h>

//using namespace memmap;
//using namespace semlock;

namespace youbot {

typedef struct {
	int jointID;
	int gearRatio;
	double maxJointValue;
	//double maxAxisValue;
	double minJointValue;
	//double minAxisValue;
	bool negative;
	double rpm_to_rad;
} YouJoint;

enum controller_mode {
	eSTOP = 0,
	ePOSITION = 1,
	eVELOCITY = 2,
	eNO_MOTOR_ACTION = 3,
	eRESET_POSITION = 4
};

//! API for controlling the youBot.
//! This API connects to the youBot-daemon,
//! So make sure that the daemon is running.

class YouBotApi {
public:
	//ctor
	YouBotApi(const char * file, int key) :
		encoderSteps(4096) {

		int ec = memMap.openMappedFile(file);
		if (ec != 0) {
			printf(
					"MapFile couldn't be opened. Check if you have permissions to read/write the file: %s",
					file);
			exit(0);
		}

		//constants for base control
		gearbox = 364.0 / 9405.0; // 0.04 ==> inverse 25.8
		wheel_radius = 0.05;
		wheel_radius_per4 = wheel_radius / 4.0;
		half_axle_length = 0.3 / 2.0;
		half_wheel_base = 0.471 / 2.0;
		geom_factor = half_axle_length + half_wheel_base;

		mappedHead
				= (soem_ethercat_drivers::YouBotHeaderMsg *) memMap.getAddr();
		mappedMsg = (soem_ethercat_drivers::YouBotSlaveMsg *) (memMap.getAddr()
				+ sizeof(soem_ethercat_drivers::YouBotHeaderMsg));
		semLock.openLock(key);

		// Joint 1 parameters
		joints[0].jointID = 1;
		joints[0].gearRatio = 156;
		joints[0].minJointValue = -169. / 180. * M_PI;
		joints[0].maxJointValue = 169. / 180. * M_PI;
		joints[0].negative = true;
		joints[0].rpm_to_rad = -0.0783;

		// Joint 2 parameters
		joints[1].jointID = 2;
		joints[1].gearRatio = 156;
		joints[1].minJointValue = -65. / 180. * M_PI;
		joints[1].maxJointValue = 90. / 180. * M_PI;
		joints[1].negative = true;
		joints[1].rpm_to_rad = -0.0807;

		// Joint 3 parameters
		joints[2].jointID = 3;
		joints[2].gearRatio = 100;
		joints[2].minJointValue = -151. / 180. * M_PI;
		joints[2].maxJointValue = 146. / 180. * M_PI;
		joints[2].negative = false;
		joints[2].rpm_to_rad = 0.0863;

		// Joint 4 parameters
		joints[3].jointID = 4;
		joints[3].gearRatio = 71;
		joints[3].minJointValue = -102.5 / 180. * M_PI;
		joints[3].maxJointValue = 102.5 / 180. * M_PI;
		joints[3].negative = true;
		joints[3].rpm_to_rad = -0.0863;

		// Joint 5 parameters
		joints[4].jointID = 5;
		joints[4].gearRatio = 71;
		joints[4].minJointValue = -167.5 / 180. * M_PI;
		joints[4].maxJointValue = 167.5 / 180. * M_PI;
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

	int setMotorPositionsOrSpeeds(int* values) {
		semLock.lock();
		mappedMsg[0].stctOutput.positionOrSpeed = values[0];
		mappedMsg[1].stctOutput.positionOrSpeed = values[1];
		mappedMsg[2].stctOutput.positionOrSpeed = values[2];
		mappedMsg[3].stctOutput.positionOrSpeed = values[3];
		mappedMsg[4].stctOutput.positionOrSpeed = values[4];
		mappedMsg[5].stctOutput.positionOrSpeed = values[5];
		mappedMsg[6].stctOutput.positionOrSpeed = values[6];
		mappedMsg[7].stctOutput.positionOrSpeed = values[7];
		mappedMsg[8].stctOutput.positionOrSpeed = values[8];
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

	int setControllerModes(int* ctrlModes) {
		semLock.lock();
		mappedMsg[0].stctOutput.controllerMode = ctrlModes[0];
		mappedMsg[1].stctOutput.controllerMode = ctrlModes[1];
		mappedMsg[2].stctOutput.controllerMode = ctrlModes[2];
		mappedMsg[3].stctOutput.controllerMode = ctrlModes[3];
		mappedMsg[4].stctOutput.controllerMode = ctrlModes[4];
		mappedMsg[5].stctOutput.controllerMode = ctrlModes[5];
		mappedMsg[6].stctOutput.controllerMode = ctrlModes[6];
		mappedMsg[7].stctOutput.controllerMode = ctrlModes[7];
		mappedMsg[8].stctOutput.controllerMode = ctrlModes[8];
		semLock.unlock();
		return 0;
	}

	/**
	 * Sets the base velocity (m/s, rad/s)
	 */
	int setBaseVelocity(double forward, double right, double yaw) {
		for (int i = 0; i < 4; i++)
			setControllerMode(i, 2);
		double gearbox = 9405.0 / 364.0;
		double forwardTicks = forward * 330000 / gearbox;
		double rightTicks = right * 346000 / gearbox;
		double yawTicks = yaw / 2 / 3.1415926 * 805000 / gearbox;

		setMotorPositionOrSpeed(0,
				(int) (-forwardTicks - rightTicks + yawTicks));
		setMotorPositionOrSpeed(1, (int) (forwardTicks - rightTicks + yawTicks));
		setMotorPositionOrSpeed(2,
				(int) (-forwardTicks + rightTicks + yawTicks));
		setMotorPositionOrSpeed(3, (int) (forwardTicks + rightTicks + yawTicks));
		return 0;
	}

	/**
	 * Move a given arm axis to a position in radians
	 * Puts the axis in position control mode !
	 * @param axis Axis number as on the robot: 1..5
	 * @param radians The desired position in radians, with zero being the
	 * joint-space null (robot complete up-right).
	 * @return zero on success, -1 if wrong axis number
	 */
	int setArmJointPosition(int axis, double radians) {
		if (axis < 1 || axis > 5)
			return -1;
		setControllerMode(axis + 3, 1);
		return setAxisPosition(axis, getAxisAbsolutePosition(axis, radians));
	}

	/**
	 * Gets the current joint position in radians
	 * @param axis Axis number as on the robot: 1..5
	 * @return the position, or zero if invalid axis.
	 */
	double getArmJointPosition(int axis) {
		if (axis < 1 || axis > 5)
			return 0;
		return getJointAbsolutePosition(axis, getAxisPosition(axis));
	}

	/**
	 * Send a given velocity setpoint to a robot's joint
	 * Puts the axis in velocity control mode !
	 * @param axis Axis number as on the robot: 1..5
	 * @param radPerSec The desired velocity in radians per second.
	 * @return zero on success, -1 if wrong axis number
	 */
	int setArmJointVelocity(int axis, double radPerSec) {
		if (axis < 1 || axis > 5)
			return -1;
		setControllerMode(axis + 3, 2);
		return setMotorPositionOrSpeed(axis + 3, radPerSecToEncoderSpeed(axis,
				radPerSec));
	}

	/**
	 * Gets the current joint velocity in radians per second
	 * @param axis Axis number as on the robot: 1..5
	 * @return the velocity, or zero if invalid axis.
	 */
	double getArmJointVelocity(int axis) {
		if (axis < 1 || axis > 5)
			return 0;
		return encoderSpeedToRadPerSec(axis, getActualVelocity(axis + 3));
	}

	int setMotorPositionsOrSpeedsAndControllerModes(int* motorValues,
			int* ctrlModes) {
		semLock.lock();

		mappedMsg[0].stctOutput.positionOrSpeed = motorValues[0];
		mappedMsg[1].stctOutput.positionOrSpeed = motorValues[1];
		mappedMsg[2].stctOutput.positionOrSpeed = motorValues[2];
		mappedMsg[3].stctOutput.positionOrSpeed = motorValues[3];
		mappedMsg[4].stctOutput.positionOrSpeed = motorValues[4];
		mappedMsg[5].stctOutput.positionOrSpeed = motorValues[5];
		mappedMsg[6].stctOutput.positionOrSpeed = motorValues[6];
		mappedMsg[7].stctOutput.positionOrSpeed = motorValues[7];
		mappedMsg[8].stctOutput.positionOrSpeed = motorValues[8];

		mappedMsg[0].stctOutput.controllerMode = ctrlModes[0];
		mappedMsg[1].stctOutput.controllerMode = ctrlModes[1];
		mappedMsg[2].stctOutput.controllerMode = ctrlModes[2];
		mappedMsg[3].stctOutput.controllerMode = ctrlModes[3];
		mappedMsg[4].stctOutput.controllerMode = ctrlModes[4];
		mappedMsg[5].stctOutput.controllerMode = ctrlModes[5];
		mappedMsg[6].stctOutput.controllerMode = ctrlModes[6];
		mappedMsg[7].stctOutput.controllerMode = ctrlModes[7];
		mappedMsg[8].stctOutput.controllerMode = ctrlModes[8];

		semLock.unlock();
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
		switch (axisNr) {
		case 0:
			if (pos < -587000)
				pos = -587000;
			break;
		case 1:
			if (pos < -269000)
				pos = -269000;
			break;
		case 2:
			if (pos < -325000)
				pos = -325000;
			break;
		case 3:
			if (pos < -162000)
				pos = -162000;
			break;
		case 4:
			if (pos < -264990)
				pos = -264990;
			break;
		}

		if (pos > 0)
			pos = 0;

		mappedMsg[axisNr + 4].stctOutput.positionOrSpeed = pos;
		semLock.unlock();
		return 0;
	}

	int setAxisPositions(int* poss) {
		semLock.lock();

		if (poss[0] < -587000)
			poss[0] = -587000;
		if (poss[1] < -269000)
			poss[1] = -269000;
		if (poss[2] < -325000)
			poss[2] = -325000;
		if (poss[3] < -162000)
			poss[3] = -162000;
		if (poss[4] < -264990)
			poss[4] = -264990;

		if (poss[0] > 0)
			poss[0] = 0;
		if (poss[1] > 0)
			poss[1] = 0;
		if (poss[2] > 0)
			poss[2] = 0;
		if (poss[3] > 0)
			poss[3] = 0;
		if (poss[4] > 0)
			poss[4] = 0;

		mappedMsg[0 + 4].stctOutput.positionOrSpeed = poss[0];
		mappedMsg[1 + 4].stctOutput.positionOrSpeed = poss[1];
		mappedMsg[2 + 4].stctOutput.positionOrSpeed = poss[2];
		mappedMsg[3 + 4].stctOutput.positionOrSpeed = poss[3];
		mappedMsg[4 + 4].stctOutput.positionOrSpeed = poss[4];

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
	int32 getActualPosition(int slaveNr) {
		return mappedMsg[slaveNr].stctInput.actualPosition;
	}
	//the caller needs to free the memory!
	int32* getActualPositions() {
		int32* positions = new int32[9];
		positions[0] = mappedMsg[0].stctInput.actualPosition;
		positions[1] = mappedMsg[1].stctInput.actualPosition;
		positions[2] = mappedMsg[2].stctInput.actualPosition;
		positions[3] = mappedMsg[3].stctInput.actualPosition;
		positions[4] = mappedMsg[4].stctInput.actualPosition;
		positions[5] = mappedMsg[5].stctInput.actualPosition;
		positions[6] = mappedMsg[6].stctInput.actualPosition;
		positions[7] = mappedMsg[7].stctInput.actualPosition;
		positions[8] = mappedMsg[8].stctInput.actualPosition;
		return positions;
	}
	int32 getActualCurrent(int slaveNr) {
		return mappedMsg[slaveNr].stctInput.actualCurrent;
	}
	int32 getActualVelocity(int slaveNr) {
		return mappedMsg[slaveNr].stctInput.actualVelocity;
	}
	int32* getActualVelocities() {
		int32* velocities = new int32[9];
		velocities[0] = mappedMsg[0].stctInput.actualVelocity;
		velocities[1] = mappedMsg[1].stctInput.actualVelocity;
		velocities[2] = mappedMsg[2].stctInput.actualVelocity;
		velocities[3] = mappedMsg[3].stctInput.actualVelocity;
		velocities[4] = mappedMsg[4].stctInput.actualVelocity;
		velocities[5] = mappedMsg[5].stctInput.actualVelocity;
		velocities[6] = mappedMsg[6].stctInput.actualVelocity;
		velocities[7] = mappedMsg[7].stctInput.actualVelocity;
		velocities[8] = mappedMsg[8].stctInput.actualVelocity;
		return velocities;
	}
	uint16 getErrorFlags(int slaveNr) {
		return mappedMsg[slaveNr].stctInput.errorFlags;
	}
	uint16 getTemparature(int slaveNr) {
		return mappedMsg[slaveNr].stctInput.driverTemperature;
	}
	int32 getAxisPosition(int axisNr) {
		return mappedMsg[axisNr + 3].stctInput.actualPosition;
	}

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


	void getBaseVelocitiesCartesian(double &vx, double &vy, double &vtheta, timeval &timestamp) {
		// get tics/second per wheel
		//numbers tickvel1..4 are according to Fig B.1. There are two mappings to be done
		// a) map these numbers from Fig B.1 to the numbers on the youBot wheels
		// b) then map these numbers to slave numbers (-1)
		// the mapping is:
		//FigB.1 number 1  -- slave number 1
		//FigB.1 number 2  -- slave number 0
		//FigB.1 number 3  -- slave number 2
		//FigB.1 number 4  -- slave number 3
		int sense_tickvel1 = mappedMsg[1].stctInput.actualVelocity;
		int sense_tickvel2 = -mappedMsg[0].stctInput.actualVelocity;
		int sense_tickvel3 = -mappedMsg[2].stctInput.actualVelocity;
		int sense_tickvel4 = mappedMsg[3].stctInput.actualVelocity;

		//make that a rad/s (wheel radius is 0.05m)
		//these v1, v2, v3, v4 are according to Figure B.1
		double sense_v1 = sense_tickvel1 / 60.0 * gearbox * (M_PI * 2.0);
		double sense_v2 = sense_tickvel2 / 60.0 * gearbox * (M_PI * 2.0);
		double sense_v3 = sense_tickvel3 / 60.0 * gearbox * (M_PI * 2.0);
		double sense_v4 = sense_tickvel4 / 60.0 * gearbox * (M_PI * 2.0);

		//now convert this to a vx,vy,vtheta
		double _vx = (-sense_v1 + sense_v2 - sense_v3 + sense_v4)
				* wheel_radius_per4;
		double _vy = (sense_v1 + sense_v2 + sense_v3 + sense_v4)
				* wheel_radius_per4;
		double _vtheta = (sense_v1 - sense_v2 - sense_v3 + sense_v4)
				* wheel_radius_per4 / geom_factor;

		vx = _vy;
		vy = _vx;
		vtheta = -_vtheta;
		timestamp = mappedHead->timestamp;

	}

	void setBaseVelocitiesCartesian(double _vx, double _vy, double _vtheta) {
		//these are the wheel velocities (rad/s) numbers acc to Fig B.1

		//TODO convert coordinate systems
		double vx = _vy;
		double vy = _vx;
		double vtheta = -_vtheta;

		double cmd_v1 = (-vx + vy + geom_factor * vtheta) / wheel_radius;
		double cmd_v2 = (vx + vy - geom_factor * vtheta) / wheel_radius;
		double cmd_v3 = (-vx + vy - geom_factor * vtheta) / wheel_radius;
		double cmd_v4 = (vx + vy + geom_factor * vtheta) / wheel_radius;

		//the above are rad/s, now convert these to tics/s and send to robot
		int cmd_tickvel1 = 60.0 * cmd_v1 / (M_PI * 2.0) / gearbox;
		int cmd_tickvel2 = 60.0 * cmd_v2 / (M_PI * 2.0) / gearbox;
		int cmd_tickvel3 = 60.0 * cmd_v3 / (M_PI * 2.0) / gearbox;
		int cmd_tickvel4 = 60.0 * cmd_v4 / (M_PI * 2.0) / gearbox;

		//todo set controller modes
		int ctrlMode1 = eVELOCITY;
		int ctrlMode2 = eVELOCITY;
		int ctrlMode3 = eVELOCITY;
		int ctrlMode4 = eVELOCITY;

		if (cmd_tickvel1 == 0)
			ctrlMode1 = eSTOP;
		if (cmd_tickvel2 == 0)
			ctrlMode2 = eSTOP;
		if (cmd_tickvel3 == 0)
			ctrlMode3 = eSTOP;
		if (cmd_tickvel4 == 0)
			ctrlMode4 = eSTOP;

		semLock.lock();
		mappedMsg[1].stctOutput.positionOrSpeed = cmd_tickvel1;
		mappedMsg[0].stctOutput.positionOrSpeed = -cmd_tickvel2;
		mappedMsg[2].stctOutput.positionOrSpeed = -cmd_tickvel3;
		mappedMsg[3].stctOutput.positionOrSpeed = cmd_tickvel4;

		mappedMsg[1].stctOutput.controllerMode = ctrlMode1;
		mappedMsg[0].stctOutput.controllerMode = ctrlMode2;
		mappedMsg[2].stctOutput.controllerMode = ctrlMode3;
		mappedMsg[3].stctOutput.controllerMode = ctrlMode4;
		semLock.unlock();
	}

	void getBasePositionCartesian(double &xPos, double &yPos, double &theta, timeval &timestamp) {
		xPos = mappedHead->stctInput.xPos;
		yPos = mappedHead->stctInput.yPos;
		theta = mappedHead->stctInput.thetaPos;	
		timestamp = mappedHead->timestamp;
	}

	// Convert from encoder increment to joint space angle (relative) in radians
	double encoderValueToJointValue(int jointID, int encoderValue) {
		return double(encoderValue) / (joints[jointID - 1].gearRatio
				* encoderSteps) * (2 * M_PI);
	}
	// Convert from joint angle (relative) in radians to encoder increment value
	int jointValueToEncoderValue(int jointID, double jointValue) {
		return jointValue / (2 * M_PI) * (joints[jointID - 1].gearRatio
				* encoderSteps);
	}

	//Convert from encoder speeds to joint speeds in radians per second
	double encoderSpeedToRadPerSec(int jointID, double encoderSpeed) {
		return encoderSpeed * joints[jointID - 1].rpm_to_rad;
	}

	//Convert from joint speeds in radians per second to encoder speeds
	double radPerSecToEncoderSpeed(int jointID, double radPerSec) {
		return radPerSec / joints[jointID - 1].rpm_to_rad;
	}

	// Convert from Joint value to axis absolute value
	int getAxisAbsolutePosition(int jointID, double jointPosition) {
		if (joints[jointID - 1].negative)
			return jointValueToEncoderValue(jointID, -jointPosition
					+ joints[jointID - 1].minJointValue);
		else
			return jointValueToEncoderValue(jointID, jointPosition
					- joints[jointID - 1].maxJointValue);
	}

	// Convert from Joint value to axis absolute value
	double getJointAbsolutePosition(int jointID, int axisPosition) {
		double value = encoderValueToJointValue(jointID, axisPosition);
		if (joints[jointID - 1].negative)
			return joints[jointID - 1].minJointValue - value;
		else
			return value + joints[jointID - 1].maxJointValue;
	}
private:
	double gearbox;
	double wheel_radius;
	double wheel_radius_per4;
	double half_axle_length;
	double half_wheel_base;
	double geom_factor;

	//semaphore-objekt
	semlock::SemaphoreLock semLock;

	//member for a memory mapped file
	memmap::MemmoryMappedFiles memMap;

	//create a msg-object-pointer and assign the address of the mapped file
	//after that use the pointer like an array
	soem_ethercat_drivers::YouBotSlaveMsg * mappedMsg;
	soem_ethercat_drivers::YouBotHeaderMsg * mappedHead;

	int encoderSteps;

	YouJoint joints[5];
};
}

#endif	/* _YOUBOT_API_H */
