/*
 * Test.cpp
 *
 *  Created on: Oct 25, 2010
 *      Author: Luca Gherardi, Peter Soetens, Ben
 */

#include "Test.h"
#include <cstdlib>
#include "YouBotArm.h"

YouBotArm arm;

Test::Test() {
	// TODO Auto-generated constructor stub

}

Test::~Test() {
	// TODO Auto-generated destructor stub
}

int main(int argc, char** argv) {

	Configuration c1;
	for(int i=0; i<5; i++){
		c1.push_back(0.0);
	}
	arm.setJointsConfiguration(c1);

	printf("Press a key");
	getchar();

	arm.setJointAbsoluteValue(0,10/180*M_PI);
	arm.setJointAbsoluteValue(1,10/180*M_PI);
	arm.setJointAbsoluteValue(2,10/180*M_PI);
	arm.setJointAbsoluteValue(3,10/180*M_PI);
	arm.setJointAbsoluteValue(4,10/180*M_PI);

	printf("Press a key");
	getchar();

	arm.setHomingPosition();

	printf("Press a key");
	getchar();

	return 0;
}
