/* 
 * File:   main.cpp
 * Author: Özgür Sen
 *
 * Created on 24. Oktober 2010, 17:32
 */

#include <cstdlib>
#include <youBotApi.h>

using namespace std;
using namespace youbot;

typedef struct{
  
  int jointID;
  int gearRatio;
  double maxJointValue;
  //double maxAxisValue;
  double minJointValue;
  //double minAxisValue;
  bool negative;
  
} Joint;

int encoderSteps = 4096; //number of steps per 2PI

Joint joints[5];

// Convert from joint value to encoder relative value 
double encoderValueToJointValue(double encoderValue, int jointID){
    return encoderValue/(joints[jointID].gearRatio*encoderSteps)*360;
}
// Convert from encoder value to joint relative value
double jointValueToEncoderValue(double jointValue, int jointID){
    return jointValue/360*(joints[jointID].gearRatio*encoderSteps);
}
 

// Convert from Joint value to axis absolute value
double getAxisAbsolutePosition(int jointPosition, int jointID){
  if(joints[jointID].negative)
      return jointValueToEncoderValue(jointPosition + joints[jointID].minJointValue,jointID);
    else
      return jointValueToEncoderValue(jointPosition - joints[jointID].maxJointValue,jointID);
}

// Convert from Joint value to axis absolute value
double getJointAbsolutePosition(int axisPosition, int jointID){
    double value = encoderValueToJointValue(axisPosition,jointID);
    if(joints[jointID].negative)
      return joints[jointID].minJointValue + value;
    else
      return value - joints[jointID].maxJointValue;
}

int main(int argc, char** argv) {

	//youBotApi
	int semaphoreKey = 12345;

	if(argc != 1) {
		semaphoreKey = atoi(argv[1]);
	}

	YouBotApi youBot("/tmp/youBotMemMapFile", semaphoreKey);

	// Joint 0 parameters
	joints[0].jointID = 0;
	joints[0].gearRatio = 156;
	joints[0].minJointValue = -169;
	joints[0].maxJointValue = 169;
	joints[0].negative = true;
	//joints[0].minAxisValue = -585659;

	// Joint 1 parameters
	joints[1].jointID = 1;
	joints[1].gearRatio = 156;
	joints[1].minJointValue = -65;
	joints[1].maxJointValue = 90;
	joints[1].negative = true;
	//joints[1].minAxisValue = -268741;

	// Joint 2 parameters
	joints[2].jointID = 2;
	joints[2].gearRatio = 100;
	joints[2].minJointValue = -151;
	joints[2].maxJointValue = 146;
	joints[2].negative = false;
	//joints[2].minAxisValue = -325596;

	// Joint 3 parameters
	joints[3].jointID = 3;
	joints[3].gearRatio = 71;
	joints[3].minJointValue = -102.5;
	joints[3].maxJointValue = 102.5;
	joints[3].negative = true;
	//joints[3].minAxisValue = -162930;

	// Joint 4 parameters
	joints[4].jointID = 4;
	joints[4].gearRatio = 71;
	joints[4].minJointValue = -167.5;
	joints[4].maxJointValue = 167.5;
	joints[4].negative = true;
	//joints[4].minAxisValue = 131697;

	/*// Compute max and min joints value
	  for{int i=0; i<5;i++}{
	  joints[i].minAxisValue = jointValueToEncoderValue(joints[i].minJointValue, joints[i].jointID);
	  joints[i].maxAxisValue = jointValueToEncoderValue(joints[i].maxJointValue, joints[i].jointID);
	  }*/


	// steer zero velocities:
	for(int i=0;i < 4; i++)
	{
		youBot.setControllerMode(i,2);  //2: velocity, 1: position, 3: move by hand
		youBot.setMotorPositionOrSpeed(i, 0);
	}

	int counter = 0;
	bool trigger = false;
	
	youBot.setAxisPosition(0, getAxisAbsolutePosition(0,0));
	youBot.setAxisPosition(1, getAxisAbsolutePosition(0,1));
	youBot.setAxisPosition(2, getAxisAbsolutePosition(0,2));
	youBot.setAxisPosition(3, getAxisAbsolutePosition(0,3));
	youBot.setAxisPosition(4, getAxisAbsolutePosition(0,4));

	while(true)
	   sleep(1);
	

return 0;
}

