#include <ros/ros.h>
#include <string>
#include <std_msgs/String.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Twist.h>
#include <cob_msgs/TactileSensor.h>
#include <geometry_msgs/Pose.h>
#include <tf_conversions/tf_kdl.h>
#include <tf/transform_listener.h>
#include <cob_srvs/Trigger.h>
#include <pthread.h>

geometry_msgs::WrenchStamped hand_calib;
geometry_msgs::WrenchStamped hand_ft;
geometry_msgs::Twist drive_last;

ros::Publisher pub;
ros::Publisher pub_drive;
btVector3 force_ee;
bool trigger_active = true;
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

/// Trigger to call this service
bool trigger(cob_srvs::Trigger::Request& request, cob_srvs::Trigger::Response& response) {
	if(trigger_active == false) trigger_active = true;
	else trigger_active = false;

	if(trigger_active) ROS_INFO("Service is active");
	else ROS_INFO("Service is inactive");
	return true;
}

/// Callback that records and filters end effector force torque
void callback_handft(geometry_msgs::WrenchStamped hand)
{
	static int i = 0;
	i++;
	if(i <= 10) {
		hand_calib = hand;
	}

	double a = 0.08;
	hand_ft.wrench.force.x = a * (hand.wrench.force.x-hand_calib.wrench.force.x) + (1-a) * hand_ft.wrench.force.x;
	hand_ft.wrench.force.y = a * (hand.wrench.force.y-hand_calib.wrench.force.y) + (1-a) * hand_ft.wrench.force.y;
	hand_ft.wrench.force.z = a * (hand.wrench.force.z-hand_calib.wrench.force.z) + (1-a) * hand_ft.wrench.force.z;

	pthread_mutex_lock(&mutex);
	force_ee.setX(hand_ft.wrench.force.x);
	force_ee.setY(hand_ft.wrench.force.y);
	force_ee.setZ(hand_ft.wrench.force.z);
	pthread_mutex_unlock(&mutex);

}

/// PD control on end effector forces which are transformed to the base
geometry_msgs::Twist controller_base(double fx, double fy, double rate)
{
	geometry_msgs::Twist drive;
	drive.linear.x = -0.008 * fx - 0.0005 * (fx-drive_last.linear.x)*rate;
	drive.linear.y = -0.008 * fy - 0.0005 * (fy-drive_last.linear.y)*rate;
	drive_last = drive;
	return drive;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cob_follow");
    ros::NodeHandle nh;

    ros::Subscriber sub_hand = nh.subscribe("/arm_controller/wrench", 10, callback_handft);
    ROS_INFO("Subscribing to %s done...", sub_hand.getTopic().c_str());

    pub_drive = nh.advertise<geometry_msgs::Twist>("/base_controller/command", 10);
    ROS_INFO("Advertised publisher %s", pub_drive.getTopic().c_str());

    pub = nh.advertise<geometry_msgs::WrenchStamped>("/follow", 10);
    ROS_INFO("Advertised publisher %s", pub.getTopic().c_str());

    ros::ServiceServer serv = nh.advertiseService("/mm/follow", trigger);
    ROS_INFO("Service %s started", serv.getService().c_str());

    force_ee.setX(0.0);
    force_ee.setY(0.0);
    force_ee.setZ(0.0);
    hand_ft.wrench.force.x = 0.0f;
    hand_ft.wrench.force.y = 0.0f;
    hand_ft.wrench.force.z = 0.0f;
    hand_ft.wrench.torque.x = 0.0f;
    hand_ft.wrench.torque.y = 0.0f;
    hand_ft.wrench.torque.z = 0.0f;

    tf::TransformListener listener;
    ros::Rate rate(10.0);

    while(nh.ok()) {
    	ros::spinOnce();
    	tf::StampedTransform transform;
    	geometry_msgs::Twist drive;
    	double fx = 0.0f, fy = 0.0f, fz = 0.0f;

    	listener.lookupTransform("/arm_7_link", "/base_footprint", ros::Time(0), transform);

    	uint32_t i = 0;
    	if(trigger_active) {
			pthread_mutex_lock(&mutex);
			/// Transform force at end effector to the base
			btVector3 axis = transform.getRotation().getAxis();
			btScalar angle = transform.getRotation().getAngle();

			force_ee.rotate(axis, angle);

			geometry_msgs::WrenchStamped force_ee_ts;
			force_ee_ts.header.stamp = ros::Time::now();
			force_ee_ts.header.seq = i++;
			force_ee_ts.header.frame_id = "/arm_7_link";
			force_ee_ts.wrench.force.x = force_ee.x();
			force_ee_ts.wrench.force.y = force_ee.y();
			force_ee_ts.wrench.force.z = force_ee.z();
			force_ee_ts.wrench.torque.x = force_ee.x();
			force_ee_ts.wrench.torque.y = force_ee.y();
			force_ee_ts.wrench.torque.z = force_ee.z();
			pub.publish(force_ee_ts);

			//force_ee.normalize();
			//ROS_INFO("%1.4lf %1.4lf %1.4lf", force_ee.getX(), force_ee.getY(), force_ee.getZ());

			fx = force_ee.getX();
			fy = force_ee.getY();
			fz = force_ee.getZ();
			pthread_mutex_unlock(&mutex);

			drive = controller_base(-fz, fy, 0.1);
			pub_drive.publish(drive);
			ROS_INFO("Drive: %.4lf %.4lf", drive.linear.x, drive.linear.y);
    	}
    }

    ROS_INFO("Done\n");
}


#if 0
    ros::Subscriber sub_tactile = nh.subscribe("/sdh_controller/tactile_data", 10, callback_tactile);
    ROS_INFO("Subscribing to %s done...", sub_tactile.getTopic().c_str());
#endif

#if 0
void callback_tactile(cob_msgs::TactileSensor sensor)
{
	std::vector<cob_msgs::TactileMatrix> sensor_pads = sensor.tactile_matrix;

	for(uint i = 0; i < sensor_pads.size(); i++) {
		printf("%d: %dx%d\n", i, sensor_pads[i].cells_x, sensor_pads[i].cells_y);
		for(uint j = 0; j < sensor_pads[i].tactile_array.size(); i++) {
			printf("%d ", sensor_pads[i].tactile_array[j]);
		}
		printf("\n");
	}
}
#endif
