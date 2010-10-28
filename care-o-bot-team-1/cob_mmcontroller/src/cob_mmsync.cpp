#include <ros/ros.h>
#include <pthread.h>

#include <geometry_msgs/Pose.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <tf_conversions/tf_kdl.h>
#include <sensor_msgs/JointState.h>
#include <cob_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include <kdl/frames_io.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/path_line.hpp>
#include <kdl/path_circle.hpp>
#include <kdl/frames.hpp>
#include <kinematics_msgs/GetPositionIK.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/trajectory_segment.hpp>

bool trigger_active = false;
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
ros::Time last;
ros::Publisher pub_arm;
ros::Publisher pub_base;
KDL::ChainFkSolverPos_recursive *fksolver1;
KDL::ChainIkSolverVel_pinv *iksolver1v;
KDL::Chain chain;
KDL::Chain chain_base_arm0;
KDL::JntArray q_virtual;
KDL::JntArray q;
KDL::Twist twist_arm;
KDL::Twist twist_base;

void controller_ik();

/// Sends command for velocity control of the arm, but both position and velocity are required by the ros topic
/// TODO: traj.points[] needs to be initialized. This fails for the first controller loop.
/// Hack: Add a count in the controller to skip the first itteration and initialize variables...
void send_vel(KDL::JntArray dq) {
	ros::Time now = ros::Time::now();
	trajectory_msgs::JointTrajectory traj;
	double horizon, dt;

	dt = now.toSec() - last.toSec();
	horizon = 3.0 * dt;
	last = now;
	traj.header.stamp = ros::Time::now() + ros::Duration(0.01);
	traj.joint_names.push_back("arm_1_joint");
	traj.joint_names.push_back("arm_2_joint");
	traj.joint_names.push_back("arm_3_joint");
	traj.joint_names.push_back("arm_4_joint");
	traj.joint_names.push_back("arm_5_joint");
	traj.joint_names.push_back("arm_6_joint");
	traj.joint_names.push_back("arm_7_joint");


	traj.points.resize(1);
	//bool nonzero = false;
	for (int i = 0; i < 7; i++) {
		//if (q_dot(i) != 0.0) {
			traj.points[0].positions.push_back(q_virtual(i) + dq(i) * horizon);
			traj.points[0].velocities.push_back(dq(i));
			q_virtual(i) += dq(i) * dt;
		//	nonzero = true;
		//}
	}
	traj.points[0].time_from_start = ros::Duration(horizon);
	//if (nonzero)
	pub_arm.publish(traj);
}

/// Parse joint states from message and record current position in q_virtual
KDL::JntArray parse_jointstates(std::vector<std::string> names, std::vector<double> positions) {
	KDL::JntArray q_temp(7);
	int count = 0;
	for (uint i = 0; i < names.size(); i++) {
		if (strncmp(names[i].c_str(), "arm_", 4) == 0) {
			q_temp(count) = positions[i];
			count++;
		}
	}
	if (!trigger_active) {
		q_virtual = q_temp;
		last = ros::Time::now();
		ROS_INFO("Starting up controller with first configuration");
	}
	return q_temp;
}

/// Callback for recording arm velocity in cartesian space
void callback_armtwist(const geometry_msgs::Twist::ConstPtr& msg) {
	twist_arm.vel.x(msg->linear.x);
	twist_arm.vel.y(msg->linear.y);
	twist_arm.vel.z(msg->linear.z);
	twist_arm.rot.x(msg->angular.x);
	twist_arm.rot.x(msg->angular.y);
	twist_arm.rot.x(msg->angular.z);

	controller_ik();
}

/// Callback for transforming base velocity to arm
void callback_basetwist(const nav_msgs::Odometry::ConstPtr& msg) {

	double vx = msg->twist.twist.linear.x;
	double vy = msg->twist.twist.linear.y;
	double omega = msg->twist.twist.angular.z;
	KDL::Frame frame_arm;
	KDL::Frame frame_armbase;

	fksolver1->JntToCart(q, frame_arm);
	//transformational part of base rotation
	KDL::Vector twist_trans = (KDL::Rotation::EulerZYX(-omega, 0.0, 0.0) * frame_arm.p) - frame_arm.p;
	//rotational part of base rotation
	KDL::ChainFkSolverPos_recursive fksolver_base_armv0(chain_base_arm0);
	fksolver_base_armv0.JntToCart(NULL, frame_armbase);
	KDL::Twist twist_rotation(twist_trans, KDL::Vector(0, 0, 0));

	twist_base.vel.x(-vx);
	twist_base.vel.y(-vy);
	twist_base += twist_rotation;

	controller_ik();
}

/// Callback for recording arm state
void callback_armstate(const pr2_controllers_msgs::JointTrajectoryControllerStateConstPtr& msg) {
	std::vector<std::string> names = msg->joint_names;
	std::vector<double> positions = msg->actual.positions;
	q = parse_jointstates(names, positions);
	controller_ik();
}

/// Controller needs to run under a mutex lock due to variables used by synchronous callbacks
void controller_ik()
{
	KDL::Frame f;
	KDL::JntArray q_out(7);
	static int count = 0;

	count++;
	if(!trigger_active) return;

	pthread_mutex_lock(&mutex);
	fksolver1->JntToCart(q, f);
	int ret = iksolver1v->CartToJnt(q, twist_arm + twist_base, q_out);
	if (ret >= 0 && count > 10)	send_vel(q_out);
	else ROS_INFO("Inverse Kinematics Failed");
	pthread_mutex_unlock(&mutex);
}

/// Trigger to call this service
bool trigger(cob_srvs::Trigger::Request& request, cob_srvs::Trigger::Response& response) {
	if (trigger_active == true) trigger_active = false;
	else trigger_active = true;

	if(trigger_active == true) ROS_INFO("Service is active");
	else ROS_INFO("Service is inactive");
	return true;
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "cob_sync_controller");
	ros::NodeHandle nh;
	KDL::Tree my_tree;
	std::string robot_desc;

	pthread_mutex_init(&mutex, NULL);

	nh.param("/robot_description", robot_desc, std::string());
	if (!kdl_parser::treeFromString(robot_desc, my_tree)) {
		ROS_ERROR("Failed to construct kdl tree");
		return false;
	}
	my_tree.getChain("base_link", "arm_7_link", chain);
	my_tree.getChain("base_link", "arm_0_link", chain_base_arm0);

	fksolver1 = new KDL::ChainFkSolverPos_recursive(chain); //Forward position solver
	iksolver1v = new KDL::ChainIkSolverVel_pinv(chain); 	//Inverse velocity solver

	ros::Subscriber sub = nh.subscribe("/arm_controller/state", 1, callback_armstate);
	ros::Subscriber cart_vel_sub = nh.subscribe("/arm_controller/cart/command", 1, callback_armtwist);
	ros::Subscriber plat_odom_sub = nh.subscribe("/base_controller/odometry", 1, callback_basetwist);
	pub_arm = nh.advertise<trajectory_msgs::JointTrajectory> ("/arm_controller/command", 1);
	pub_base = nh.advertise<geometry_msgs::Twist> ("/base_controller/command", 1);

	ros::ServiceServer serv = nh.advertiseService("/mm/sync", trigger);
	ROS_INFO("Running cartesian velocity controller.");
	ros::spin();

	return 0;
}
