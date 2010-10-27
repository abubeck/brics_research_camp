#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "youbot_hal.h"
#include "time.h"

    nav_msgs::Odometry youbot_msg;
    youbot_hal::youbot_movement_command youbot_command;
    tf::TransformBroadcaster* tfb;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void commandCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	ROS_INFO("I heard command vx, vy, vtheta: [%f, %f, %f]", msg->linear.x, msg->linear.y, msg->angular.z);
	youbot_command.vel_x = msg->linear.x;
	youbot_command.vel_y = msg->linear.y;
	youbot_command.vel_theta = msg->angular.z;

        // Convert to ROS convention, making Y increase to the robot's left, and Z up
	youbot_command.vel_y = -youbot_command.vel_y;
	youbot_command.vel_theta = -youbot_command.vel_theta; 
}

youBotHal::youBotHal() {
    gearbox = 364.0 / 9405.0; // 0.04 ==> inverse 25.8
    tics2rad =  1.0 / 4096.0; //
    wheel_radius = 0.05;
    wheel_radius_per4 = wheel_radius/4.0;
    half_axle_length = 0.3 / 2.0;
    half_wheel_base = 0.471 / 2.0;
	geom_factor = half_axle_length + half_wheel_base;
}



youBotHal::~youBotHal() {

}

int youBotHal::initYoubotControllers(int semaphoreKey, int arm_mode, int platform_mode)
{
	int error = 0;

	youBot = new youbot::YouBotApi("/tmp/youBotMemMapFile", semaphoreKey);




	// Platform Controllers
	if( youBot->setControllerMode(0, platform_mode) != 0) {
		ROS_ERROR("ERROR in Controller 0");
		error = error - 1;
	}
	if( youBot->setControllerMode(1, platform_mode) != 0) {
		ROS_ERROR("ERROR in Controller 1");
		error = error - 2;
	}
	if( youBot->setControllerMode(2, platform_mode) != 0) {
		ROS_ERROR("ERROR in Controller 2");
		error = error - 4;
	}
	if( youBot->setControllerMode(3, platform_mode) != 0) {
		ROS_ERROR("ERROR in Controller 3");
		error = error - 8;
	}
	// Arm Controllers
	if( youBot->setControllerMode(0, arm_mode) != 0) {
		ROS_ERROR("ERROR in Controller 0");
		error = error - 16;
	}
	if( youBot->setControllerMode(0, arm_mode) != 0) {
		ROS_ERROR("ERROR in Controller 0");
		error = error - 32;
	}
	if( youBot->setControllerMode(0, arm_mode) != 0) {
		ROS_ERROR("ERROR in Controller 0");
		error = error - 64;
	}
	if( youBot->setControllerMode(0, arm_mode) != 0) {
		ROS_ERROR("ERROR in Controller 0");
		error = error - 128;
	}
	if( youBot->setControllerMode(0, arm_mode) != 0) {
		ROS_ERROR("ERROR in Controller 0");
		error = error - 256;
	}

	return error;
}

void youBotHal::sense(nav_msgs::Odometry& youbot_msg) {

	timeval timestamp;
	youBot->getBaseVelocitiesCartesian(youbot_msg.twist.twist.linear.x, youbot_msg.twist.twist.linear.y, youbot_msg.twist.twist.angular.z, timestamp);

        ros::Time rostime;
	rostime.sec = timestamp.tv_sec;
	rostime.nsec = timestamp.tv_usec*1000;
	youbot_msg.header.stamp = rostime;

        double yaw;
	youBot->getBasePositionCartesian(youbot_msg.pose.pose.position.x, youbot_msg.pose.pose.position.y, yaw, timestamp);
	// create quaternion
        tf::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        youbot_msg.pose.pose.orientation.x = q.x();
        youbot_msg.pose.pose.orientation.y = q.y();
        youbot_msg.pose.pose.orientation.z = q.z();
        youbot_msg.pose.pose.orientation.w = q.w();

        // Also create and publish tf data
        tf::StampedTransform tx(tf::Transform(q,tf::Vector3(youbot_msg.pose.pose.position.x,
                                              youbot_msg.pose.pose.position.y,
                                              youbot_msg.pose.pose.position.z)),
                                rostime, "odom", "base_link");
        tfb->sendTransform(tx);

 return;
	// get tics/second per wheel
	//numbers tickvel1..4 are according to Fig B.1. There are two mappings to be done
	// a) map these numbers from Fig B.1 to the numbers on the youBot wheels
	// b) then map these numbers to slave numbers (-1)
	// the mapping is:
	//FigB.1 number 1  -- slave number 1
	//FigB.1 number 2  -- slave number 0
	//FigB.1 number 3  -- slave number 2
	//FigB.1 number 4  -- slave number 3
	int sense_tickvel1 =  youBot->getActualVelocity(1);
	int sense_tickvel2 = -youBot->getActualVelocity(0);
	int sense_tickvel3 = -youBot->getActualVelocity(2);
	int sense_tickvel4 =  youBot->getActualVelocity(3);

	ROS_INFO("SENSED    Ticks: Tick1: %d, Tick2: %d, Tick3: %d, Tick4: %d", sense_tickvel1, sense_tickvel2, sense_tickvel3, sense_tickvel4);

	//make that a rad/s (wheel radius is 0.05m)
	//these v1, v2, v3, v4 are according to Figure B.1
	double  sense_v1 = sense_tickvel1 / 60.0 * gearbox * (M_PI * 2.0);
	double  sense_v2 = sense_tickvel2 / 60.0 * gearbox * (M_PI * 2.0);
	double  sense_v3 = sense_tickvel3 / 60.0 * gearbox * (M_PI * 2.0);
	double  sense_v4 = sense_tickvel4 / 60.0 * gearbox * (M_PI * 2.0);

	//now convert this to a vx,vy,vtheta

	youbot_msg.twist.twist.linear.x = (-sense_v1+sense_v2-sense_v3+sense_v4)*wheel_radius_per4;
	youbot_msg.twist.twist.linear.y = ( sense_v1+sense_v2+sense_v3+sense_v4)*wheel_radius_per4;
	youbot_msg.twist.twist.angular.z = ( sense_v1-sense_v2-sense_v3+sense_v4)*wheel_radius_per4/geom_factor;

        // Convert to ROS convention, making Y increase to the robot's left, and Z up
	youbot_msg.twist.twist.linear.y = -youbot_msg.twist.twist.linear.y;
	youbot_msg.twist.twist.angular.z = -youbot_msg.twist.twist.angular.z;


	//integrate to global odometry pose
	//how: little circular movements? I guess....
	//hint: you can always infer theta from the wheel position absoute. but this gets lost when the fist slipping occurs
}


int youBotHal::act(youbot_hal::youbot_movement_command youbot_command)
{


	youBot->setBaseVelocitiesCartesian(youbot_command.vel_x, youbot_command.vel_y, youbot_command.vel_theta);


	int error = 0;
//	//set velocities
//	//these are the wheel velocities (rad/s) numbers acc to Fig B.1
//
//	double cmd_v1 = (-youbot_command.vel_x + youbot_command.vel_y + geom_factor*youbot_command.vel_theta)/wheel_radius;
//	double cmd_v2 = ( youbot_command.vel_x + youbot_command.vel_y - geom_factor*youbot_command.vel_theta)/wheel_radius;
//	double cmd_v3 = (-youbot_command.vel_x + youbot_command.vel_y - geom_factor*youbot_command.vel_theta)/wheel_radius;
//	double cmd_v4 = ( youbot_command.vel_x + youbot_command.vel_y + geom_factor*youbot_command.vel_theta)/wheel_radius;
//
//	//the above are rad/s, now convert these to tics/s and send to robot
//	int cmd_tickvel1 = 60.0 * cmd_v1 / (M_PI * 2.0) / gearbox;
//	int cmd_tickvel2 = 60.0 * cmd_v2 / (M_PI * 2.0) / gearbox;
//	int cmd_tickvel3 = 60.0 * cmd_v3 / (M_PI * 2.0) / gearbox;
//	int cmd_tickvel4 = 60.0 * cmd_v4 / (M_PI * 2.0) / gearbox;
//
//	ROS_INFO("COMMANDED Ticks: Tick1: %d, Tick2: %d, Tick3: %d, Tick4: %d", cmd_tickvel1, cmd_tickvel2, cmd_tickvel3, cmd_tickvel4);
//
//	//TODO:make sure that controller mode is set to velocity for the wheels
//	if( youBot->setMotorPositionOrSpeed(1,  cmd_tickvel1) != 0) {
//		ROS_ERROR("ERROR while setting speed of controller 1");
//		error -= 1;
//	}
//	if( youBot->setMotorPositionOrSpeed(0, -cmd_tickvel2) != 0) {
//		ROS_ERROR("ERROR while setting speed of controller 0");
//		error -= 2;
//	}
//	if( youBot->setMotorPositionOrSpeed(2, -cmd_tickvel3) != 0) {
//		ROS_ERROR("ERROR while setting speed of controller 2");
//		error -= 4;
//	}
//	if( youBot->setMotorPositionOrSpeed(3,  cmd_tickvel4) != 0) {
//		ROS_ERROR("ERROR while setting speed of controller 3");
//		error -= 8;
//	}

	return error;
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line. For programmatic
	 * remappings you can use a different version of init() which takes remappings
	 * directly, but for most command-line programs, passing argc and argv is the easiest
	 * way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "youbot_hal");
        tfb = new tf::TransformBroadcaster();

	youBotHal hal;
	hal.initYoubotControllers(11111, youbot::eVELOCITY, youbot::eVELOCITY);

	/**
	 * NodeHandle is the main access point to communications with the ROS system.
	 * The first NodeHandle constructed will fully initialize this node, and the last
	 * NodeHandle destructed will close down the node.
	 */
	ros::NodeHandle n;



	//ros::Publisher youbot_hal_pub = n.advertise<youbot_hal::youbot_state>("youbot_hal_state", 1000);
	ros::Publisher youbot_hal_pub = n.advertise<nav_msgs::Odometry>("youbot_odometry", 1000);
	//ros::Subscriber youbot_hal_sub = n.subscribe("youbot_movement_command", 1000, commandCallback);
	ros::Subscriber youbot_hal_sub = n.subscribe("base_controller/command", 1000, commandCallback);


	ros::Rate loop_rate(10);

	/**
	 * A count of how many messages we have sent. This is used to create
	 * a unique string for each message.
	 */
	int count = 0;






	while (ros::ok())
	{

		hal.sense(youbot_msg);


		// Test for distance measurement
//		sleep(5);
//		youbot_command.vel_y = 0.1;
//		youbot_command.vel_x = 0;
//		youbot_command.vel_theta = 0;
//		hal.act(youbot_command);
//		sleep(10);
//		youbot_command.vel_y = 0;
//		youbot_command.vel_x = 0;
//		youbot_command.vel_theta = 0;

		hal.act(youbot_command);



		std::stringstream ss;

		ROS_INFO("COMMANDED Vel x: %f Vel y: %f Vel theta: %f", youbot_command.vel_x, youbot_command.vel_y, youbot_command.vel_theta);
		ROS_INFO("SENSED    Vel x: %f Vel y: %f Vel theta: %f", youbot_msg.twist.twist.linear.x, youbot_msg.twist.twist.linear.y, youbot_msg.twist.twist.angular.z);


		/**
		 * The publish() function is how you send messages. The parameter
		 * is the message object. The type of this object must agree with the type
		 * given as a template parameter to the advertise<>() call, as was done
		 * in the constructor above.
		 */
		youbot_hal_pub.publish(youbot_msg);

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}


	return 0;
}

