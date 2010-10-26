
#ifndef YOUBOT_HAL_H
#define	YOUBOT_HAL_H

#include "ros/ros.h"
#include "std_msgs/String.h"


#include "youbot_hal/youbot_movement_command.h"
#include "youbot_hal/youbot_state.h"

#include "../../youBotApi/include/youBotApi.h"

#include <sstream>

class youBotHal {

public:
	youBotHal();
	~youBotHal();
	int initYoubotControllers(int semaphoreKey, int arm_mode, int platform_mode);
    int act(youbot_hal::youbot_movement_command youbot_command);
    void sense(youbot_hal::youbot_state& youbot_msg);

private:

	youbot::YouBotApi* youBot;
    // consts

    double gearbox;
    double tics2rad;
    double wheel_radius;
    double wheel_radius_per4;
    double half_axle_length;
    double half_wheel_base;
    double geom_factor;

};

#endif	/* CONTROLLER_H */

