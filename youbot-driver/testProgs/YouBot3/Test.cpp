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

	system("pause");

	arm.setJointAbsoluteValue(0,10);
	arm.setJointAbsoluteValue(1,10);
	arm.setJointAbsoluteValue(2,10);
	arm.setJointAbsoluteValue(3,10);
	arm.setJointAbsoluteValue(4,10);

	system("pause");

	arm.setHomingPosition();

	while(1)
		sleep(1);

	return 0;
}
