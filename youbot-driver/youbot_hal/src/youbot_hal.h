
#ifndef YOUBOT_HAL_H
#define	YOUBOT_HAL_H

#include "ros/ros.h"
#include "std_msgs/String.h"


#include "youbot_hal/youbot_movement_command.h"
#include "youbot_hal/youbot_state.h"

#include "../../youBotApi/include/youBotApi.h"

#include <sstream>

namespace youbot_hal {

class youbot_hal {

public:
	int initYoubotControllers(int semaphoreKey)
    int act();
    int sense();

private:

	youbot::YouBotApi* youBot;
    // consts

    const double gearbox =  9405.0 / 364.0 ; // 0.04
    const double tics2rad =  1.0 / 4096.0; //
    const double wheel_radius = 0.05;
    const double wheel_radius_per4 = wheel_radius/4.0;
    const double half_axle_length = 0.3 / 2.0;
    const double half_wheel_base = 0.471 / 2.0;

};

} // namespace

#endif	/* CONTROLLER_H */

