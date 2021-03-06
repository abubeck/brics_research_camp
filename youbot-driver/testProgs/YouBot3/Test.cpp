/*
 * Test.cpp
 *
 *  Created on: Oct 25, 2010
 *      Author: Luca Gherardi, Peter Soetens,  Benjamin Rosman
 */

#include "Test.h"
#include <cstdlib>
#include "YouBotArm.h"
#include "math.h"
//#include "JointValue.h"
//#include "JointValues.h"

YouBotArm arm;

Test::Test() {
	// TODO Auto-generated constructor stub

}

Test::~Test() {
	// TODO Auto-generated destructor stub
}

int main(int argc, char** argv) {

	JointValues configuration;

	for(int i=0; i<5; i++){
		JointValue j;
		j.joint_uri = i;
		j.value = 0.0;
		configuration.values.push_back(j);
	}
	printf("OK");
	arm.setJointsConfiguration(configuration);

	printf("Press a key");
	getchar();

	JointValue joint1;
	joint1.joint_uri = "0";
	joint1.value = 20./180.*M_PI;
	arm.setJointAbsoluteValue(joint1);

	JointValue joint2;
	joint2.joint_uri = "3";
	joint2.value = 90./180.*M_PI;
	arm.setJointAbsoluteValue(joint2);

	printf("Press a key");
	getchar();

	arm.setHomingPosition();

	printf("Press a key");
	getchar();


	/*
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
	*/

	return 0;
}
