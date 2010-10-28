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

ros::Publisher pub_drive;
btVector3 force_ee;
bool trigger_active = false;
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

bool trigger(cob_srvs::Trigger::Request& request, cob_srvs::Trigger::Response& response) {
	if(trigger_active == false) trigger_active = true;
	else trigger_active = false;

	if(trigger_active) ROS_INFO("Service is active");
	else ROS_INFO("Service is inactive");
	return true;
}

void callback_handft(geometry_msgs::WrenchStamped hand)
{
	static int _hand_count = 0;
	_hand_count++;
	if(_hand_count <= 1) {
		hand_calib = hand;
	}

	double a = 0.08;
	hand_ft.wrench.force.x = a * (hand.wrench.force.x - hand_calib.wrench.force.x) + (1-a) * hand_ft.wrench.force.x;
	hand_ft.wrench.force.y = a * (hand.wrench.force.y - hand_calib.wrench.force.y) + (1-a) * hand_ft.wrench.force.y;
	hand_ft.wrench.force.z = a * (hand.wrench.force.z - hand_calib.wrench.force.z) + (1-a) * hand_ft.wrench.force.z;

	pthread_mutex_lock(&mutex);
	force_ee.setX(0.1 * hand_ft.wrench.force.x);
	force_ee.setY(0.1 * hand_ft.wrench.force.y);
	force_ee.setZ(0.1 * hand_ft.wrench.force.z);
	pthread_mutex_unlock(&mutex);
}

geometry_msgs::Twist controller_base(double fx, double fy, double rate)
{
	geometry_msgs::Twist drive;
	drive.linear.x = -0.1 * fx - 0.005 * (fx-drive_last.linear.x)*rate;
	drive.linear.y = -0.1 * fy - 0.005 * (fy-drive_last.linear.y)*rate;
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

    ros::ServiceServer serv = nh.advertiseService("/mm/follow", trigger);
    ROS_INFO("Service %s started", serv.getService().c_str());

    tf::TransformListener listener;
    ros::Rate rate(10.0);

    while(nh.ok()) {
    	tf::StampedTransform transform;
    	geometry_msgs::Twist drive;
    	double fx = 0.0f, fy = 0.0f;

    	listener.lookupTransform("/base_footprint", "/arm_7_link", ros::Time(0), transform);

    	pthread_mutex_lock(&mutex);
    	force_ee.rotate(transform.getRotation().getAxis(), transform.getRotation().getAngle());
    	force_ee.normalize();
    	fx = force_ee.getX();
    	fy = force_ee.getY();
    	pthread_mutex_unlock(&mutex);

    	drive = controller_base(fx, fy, 0.1);
    	pub_drive.publish(drive);
    	ROS_INFO("Drive: %.4lf %.4lf", drive.linear.x, drive.linear.y);
    }

    ros::spin();
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
