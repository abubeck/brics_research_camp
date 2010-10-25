/*
 * Manipulator.h
 *
 *  Created on: Oct 25, 2010
 *      Author: luca
 */

#ifndef MANIPULATOR_H_
#define MANIPULATOR_H_

#include <cstdlib>
#include <stdlib.h>
#include <vector>

typedef struct{

  int jointID; //Joint ID
  int gearRatio; // Joint gear ratio: number of motor
  double maxJointValue;
  double minJointValue;
  bool negative;

}Joint;

typedef std::vector<double> Configuration;

class Manipulator {
public:
	Manipulator();
	virtual ~Manipulator();
	virtual int setJointAbsoluteValue(int jointID, double value)=0;
	virtual void setJointsConfiguration(Configuration config);
};



#endif /* MANIPULATOR_H_ */
