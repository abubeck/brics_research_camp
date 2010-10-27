#include <ros/ros.h>
#include <string>

#include <kinematics_msgs/GetPositionIK.h>

#include <geometry_msgs/Pose.h>
#include <tf_conversions/tf_kdl.h>
#include <tf/transform_listener.h>

#include <cob_srvs/Trigger.h>

#include <visualization_msgs/Marker.h>
#include <actionlib/client/action_client.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <cob_msgs/MoveAction.h>
#include <iostream>

class RobotOps
{
public:

    typedef actionlib::ActionClient<pr2_controllers_msgs::JointTrajectoryAction> ArmJointAction;
    typedef actionlib::ActionClient<move_base_msgs::MoveBaseAction> BaseAction;
    typedef actionlib::ActionClient<cob_msgs::MoveAction> MoveAction;
    
    RobotOps(void) : node_("~")
    {
	visualizationMarkerPublisher_ = node_.advertise<visualization_msgs::Marker>("follow", 10240);
	moving_ = false;
    }

    void readParamTrajectory(const std::string &name, trajectory_msgs::JointTrajectory &traj)
    {
	traj.points.clear();
	
	ROS_INFO("Getting trajectory %s from param server", name.c_str());
	
	XmlRpc::XmlRpcValue v;
	if (node_.hasParam(name))
	    node_.getParam(name, v);
	else
	    ROS_ERROR("Trajectory %s not set", name.c_str());

	traj.joint_names = armJointName_;
	traj.points.resize(v.size());
	
	std::cout  << v.size() << " states on traj" << std::endl;
	
	for (int i = 0 ; i < v.size() ; ++i)
	{
	    XmlRpc::XmlRpcValue vi = v[i];
	    std::cout << vi.size() << std::endl;
	    
	    traj.points[i].positions.resize(7);
	    if (vi.size() != 7)
		ROS_ERROR("7 joint values expected per point");
	    
	    for (int j = 0 ; j < vi.size() ; ++j)
	    {
		double x;
		try
		{
		    x = (double)vi[j];
		}
		catch(...)
		{
		    int xi = (int)vi[j];
		    x = xi;
		}

		traj.points[i].positions[j] = x;
	    }
	    traj.points[i].time_from_start = ros::Duration(5 * (i + 1));
	}
    }
    
    bool configure(void)
    {
	if (!configureArm())
	    return false;
	if (!configureBase())
	    return false;

	if (!configureMove())
	    return false;

	return true;
	
	
	/*
	std::string robot_desc_string;
	node_.param("/robot_description", robot_desc_string, std::string());
	if (!kdl_parser::treeFromString(robot_desc_string, tree_))
	{
	    ROS_ERROR("Failed to construct kdl tree");
	    return false;
	}
	else
	{
	    tree_.getChain("base_link","arm_7_link", chain_);
	    return true;
	}
	*/
    }
    
    bool configureArm(void)
    {
	armAction_.reset(new ArmJointAction("arm_controller/joint_trajectory_action"));

	armJointName_.resize(7);
	armJointName_[0] = "arm_1_joint";
	armJointName_[1] = "arm_2_joint";
	armJointName_[2] = "arm_3_joint";
	armJointName_[3] = "arm_4_joint";
	armJointName_[4] = "arm_5_joint";
	armJointName_[5] = "arm_6_joint";
	armJointName_[6] = "arm_7_joint";
	
	while(!armAction_->waitForActionServerToStart(ros::Duration(1.0)))
	{
	    ROS_INFO("Waiting for the arm_controller/joint_trajectory_action server to come up.");
	    ros::spinOnce();
	    if (!node_.ok())
		return false;
	    
	}
	ROS_INFO("Connected to the arm controller");
	return true;
    }
    
    bool configureBase(void)
    {	
	// configure base
	baseAction_.reset(new BaseAction("move_base"));
	while(!baseAction_->waitForActionServerToStart(ros::Duration(1.0)))
	{
	    ROS_INFO("Waiting for the base_action server to come up.");
	    ros::spinOnce();
	    if (!node_.ok())
		return false;
	    
	}
	ROS_INFO("Connected to the base controller");
	return true;
    }

    bool configureMove(void)
    {	
	// configure base
	moveAction_.reset(new MoveAction("script_server"));
	while(!moveAction_->waitForActionServerToStart(ros::Duration(1.0)))
	{
	    ROS_INFO("Waiting for the move_action server to come up.");
	    ros::spinOnce();
	    if (!node_.ok())
		return false;
	    
	}
	ROS_INFO("Connected to the sript server");
	return true;
    }

    void move(const std::string &comp, const std::string &location)
    {
	cob_msgs::MoveGoal goal;  
	goal.component_name = comp;
	goal.parameter_name = location;
	ROS_INFO("Moving %s to %s", comp.c_str(), location.c_str());
	moving_ = true;
	moveGoalHandle_ = moveAction_->sendGoal(goal, boost::bind(&RobotOps::moveTransitionCallback, this, _1));
    }
    
    void moveArm(const std::string &name)
    {	
	trajectory_msgs::JointTrajectory t;
	readParamTrajectory("/script_server/arm/" + name, t);
	if (!t.points.empty())
	    moveArm(t);
	else
	    ROS_ERROR("No points on trajectory");
    }
    
    void armHome(void)
    {
	trajectory_msgs::JointTrajectory home;
	home.joint_names = armJointName_;
	home.points.resize(1);
	home.points[0].positions.resize(7);
	home.points[0].positions[0] = 0;
	home.points[0].positions[1] = 0;
	home.points[0].positions[2] = 0;
	home.points[0].positions[3] = 0;
	home.points[0].positions[4] = 0;
	home.points[0].positions[5] = 0;
	home.points[0].positions[6] = 0;
	home.points[0].time_from_start = ros::Duration(3);
	
	moveArm(home);
    }

    std::vector<double> armIK(const geometry_msgs::PoseStamped &pose)
    {
	std::vector<double> r;

	if (!ros::service::waitForService("/arm_controller/get_ik", ros::Duration(2.0)))
	{
	    ROS_ERROR("IK service not found");
	    return r;
	}
	ros::ServiceClient ik_client = node_.serviceClient<kinematics_msgs::GetPositionIK>("/arm_controller/get_ik");

	// define the service messages
	kinematics_msgs::GetPositionIK::Request  gpik_req;
	kinematics_msgs::GetPositionIK::Response gpik_res;
	gpik_req.timeout = ros::Duration(0.5);
	
	// convert the desired pose for the end effector into corresponding message
	gpik_req.ik_request.pose_stamped = pose;
	
	// fill in joint names and initial (random) positions
	gpik_req.ik_request.ik_seed_state.joint_state.position.resize(7);
	//	gpik_req.ik_request.ik_seed_state.joint_state.name = responseSolverInfo.kinematic_solver_info.joint_names;
	for(unsigned int i = 0; i < 7 ; ++i)
	    gpik_req.ik_request.ik_seed_state.joint_state.position[i] = (((double)rand() / (double)RAND_MAX));
	
	// call the IK service
	if (ik_client.call(gpik_req, gpik_res))
	{
	    if (gpik_res.error_code.val == gpik_res.error_code.SUCCESS)
		r = gpik_res.solution.joint_state.position;
	    else
		ROS_ERROR("IK Failed");
	}
	else
	{
	    ROS_ERROR("Unable to call IK service");	    
	}
	
	return r;
    }

    void trigger(void)
    {
	ros::ServiceClient stopArm = node_.serviceClient<cob_srvs::Trigger>("/mm/run");
	cob_srvs::Trigger::Request req;
	cob_srvs::Trigger::Response resp;
	if (stopArm.call(req, resp))
	{
	    ROS_INFO("Stopping arm in cart space");
	}
    }
    
    
    void testIK(void)
    {
	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = "/base_link";
	pose.header.stamp = ros::Time::now();
	pose.pose.position.x = 3 * ((double)rand() / (double)RAND_MAX - 0.5);
	pose.pose.position.y = 3 * ((double)rand() / (double)RAND_MAX - 0.5);
	pose.pose.position.z = 3 * ((double)rand() / (double)RAND_MAX - 0.5);
	//	std::cout << pose.pose.position.x << std::endl << pose.pose.position.y << std::endl << pose.pose.position.z << std::endl;
	
	pose.pose.orientation.x = 0;	
	pose.pose.orientation.y = 0;
	pose.pose.orientation.z = 0;
	pose.pose.orientation.w = 1;
	
	std::vector<double> r = armIK(pose);
	for (unsigned int i = 0 ; i < r.size() ; ++i)
	    std::cout << r[i] << " ";
	std::cout << std::endl;
    }

    bool moveArmCart(const geometry_msgs::PoseStamped &pose)
    {
	geometry_msgs::PoseStamped poseBase;
	tf_.transformPose("/base_link", pose, poseBase);
	std::vector<double> ik = armIK(poseBase);
	if (!ik.empty())
	{
	    moveArm(ik);
	    return true;
	}
	else
	    return false;
    }
    
    void moveArm(std::vector<double> &pos)
    {
	trajectory_msgs::JointTrajectory home;
	home.joint_names = armJointName_;
	home.points.resize(1);
	home.points[0].positions = pos;
	home.points[0].time_from_start = ros::Duration(3);
	moveArm(home);
    }
    
    void moveArm(const trajectory_msgs::JointTrajectory &trajectory)
    {
	pr2_controllers_msgs::JointTrajectoryGoal goal;  
	goal.trajectory = trajectory;
	goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.2);
	
	ROS_INFO("Sending trajectory with %d points and timestamp: %f",(int)goal.trajectory.points.size(),
		 goal.trajectory.header.stamp.toSec());
	moving_ = true;
	armGoalHandle_ = armAction_->sendGoal(goal, boost::bind(&RobotOps::armTransitionCallback, this, _1));
    }
    
    void baseHome(void)
    {
	geometry_msgs::PoseStamped pose;
	pose.pose.position.x = 0;
	pose.pose.position.y = 0;
	pose.pose.position.z = 0;

	pose.pose.orientation.x = 0;
	pose.pose.orientation.y = 0;
	pose.pose.orientation.z = 0;
	pose.pose.orientation.w = 1;

	pose.header.stamp = ros::Time::now();
	pose.header.frame_id = "map";
	
	moveBase(pose);	
    }

    void moveBase(const std::string &nm)
    {
	std::string name = "/script_server/base/" + nm;

	ROS_INFO("Getting base pos %s from param server", name.c_str());
	
	XmlRpc::XmlRpcValue v;
	if (node_.hasParam(name))
	    node_.getParam(name, v);
	else
	    ROS_ERROR("Base position %s not set", name.c_str());

	if (v.size() != 3)
	{
	    ROS_ERROR("Not a base position");
	    return;
	}
	
	double hh[3];
	for (int i = 0 ; i < v.size() ; ++i)
	{
	    XmlRpc::XmlRpcValue vi = v[i];
	    double x;
	    try
	    {
		x = (double)vi;
	    }
	    catch(...)
	    {
		int xi = (int)vi;
		x = xi;
	    }
	    hh[i] = x;
	}
	geometry_msgs::PoseStamped pose;
	pose.pose.position.x = hh[0];
	pose.pose.position.y = hh[1];
	pose.pose.position.z = 0;
	
	tf::quaternionTFToMsg(tf::createQuaternionFromYaw(hh[2]), pose.pose.orientation);
	
			  
	pose.header.stamp = ros::Time::now();
	pose.header.frame_id = "/map";
	moveBase(pose);
    }
    
    void moveBase(const geometry_msgs::PoseStamped &pose)
    {
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose = pose;
	
	ROS_INFO("Sending base goal");
	moving_ = true;
	baseGoalHandle_ = baseAction_->sendGoal(goal, boost::bind(&RobotOps::baseTransitionCallback, this, _1));
    }
    
    void armTransitionCallback(ArmJointAction::GoalHandle gh) 
    {   
	if (gh != armGoalHandle_)
	    return;
	actionlib::CommState comm_state = gh.getCommState();
	switch (comm_state.state_)
	{
	case actionlib::CommState::WAITING_FOR_GOAL_ACK:
	case actionlib::CommState::PENDING:
	case actionlib::CommState::RECALLING:
	    return;
	case actionlib:: CommState::ACTIVE:
	case actionlib::CommState::PREEMPTING:
	    return;
	case actionlib::CommState::DONE:
	    {
		switch(gh.getTerminalState().state_)
		{
		case actionlib::TerminalState::RECALLED:
		case actionlib::TerminalState::REJECTED:
		case actionlib::TerminalState::PREEMPTED:
		case actionlib::TerminalState::ABORTED:
		case actionlib::TerminalState::LOST:
		    {
			ROS_INFO("Arm trajectory controller status came back as failed");
			armGoalHandle_.reset();
			moving_ = false;
			return;
		    }
		case actionlib::TerminalState::SUCCEEDED:
		    {
			ROS_INFO("Arm trajectory controller status came back as succeeded");
			armGoalHandle_.reset();
			moving_ = false;
			return;
		    }
		default:
		    ROS_ERROR("Unknown terminal state [%u]. This is a bug in ActionClient", gh.getTerminalState().state_);
		}
	    }
	default:
	    break;
	}
    } 

    void baseTransitionCallback(BaseAction::GoalHandle gh) 
    {   
	if (gh != baseGoalHandle_)
	    return;
	actionlib::CommState comm_state = gh.getCommState();
	switch (comm_state.state_)
	{
	case actionlib::CommState::WAITING_FOR_GOAL_ACK:
	case actionlib::CommState::PENDING:
	case actionlib::CommState::RECALLING:
	    //	    controller_status_ = QUEUED;
	    return;
	case actionlib:: CommState::ACTIVE:
	case actionlib::CommState::PREEMPTING:
	    //	    controller_status_ = ACTIVE;
	    return;
	case actionlib::CommState::DONE:
	    {
		switch(gh.getTerminalState().state_)
		{
		case actionlib::TerminalState::RECALLED:
		case actionlib::TerminalState::REJECTED:
		case actionlib::TerminalState::PREEMPTED:
		case actionlib::TerminalState::ABORTED:
		case actionlib::TerminalState::LOST:
		    {
			ROS_INFO("Base controller status came back as failed");
			baseGoalHandle_.reset();
			moving_ = false;
			return;
		    }
		case actionlib::TerminalState::SUCCEEDED:
		    {
			ROS_INFO("Base controller status came back as succeeded");
			baseGoalHandle_.reset();
			moving_ = false;
		    }
		default:
		    ROS_ERROR("Unknown terminal state [%u]. This is a bug in ActionClient", gh.getTerminalState().state_);
		}
	    }
	default:
	    break;
	}
    } 

    void moveTransitionCallback(MoveAction::GoalHandle gh) 
    {   
	if (gh != moveGoalHandle_)
	    return;
	actionlib::CommState comm_state = gh.getCommState();
	switch (comm_state.state_)
	{
	case actionlib::CommState::WAITING_FOR_GOAL_ACK:
	case actionlib::CommState::PENDING:
	case actionlib::CommState::RECALLING:
	    //	    controller_status_ = QUEUED;
	    return;
	case actionlib:: CommState::ACTIVE:
	case actionlib::CommState::PREEMPTING:
	    //	    controller_status_ = ACTIVE;
	    return;
	case actionlib::CommState::DONE:
	    {
		switch(gh.getTerminalState().state_)
		{
		case actionlib::TerminalState::RECALLED:
		case actionlib::TerminalState::REJECTED:
		case actionlib::TerminalState::PREEMPTED:
		case actionlib::TerminalState::ABORTED:
		case actionlib::TerminalState::LOST:
		    {
			ROS_INFO("Arm trajectory controller status came back as failed");
			moving_ = false;
			armGoalHandle_.reset();
			return;
		    }
		case actionlib::TerminalState::SUCCEEDED:
		    {
			ROS_INFO("Arm trajectory controller status came back as succeeded");
 			moving_ = false;
			armGoalHandle_.reset();
			return;
		    }
		default:
		    ROS_ERROR("Unknown terminal state [%u]. This is a bug in ActionClient", gh.getTerminalState().state_);
		}
	    }
	default:
	    break;
	}
    } 
    
    void monitorBasePos(void)
    {
	tf::StampedTransform out;
	while (1)
	{
	    tf_.lookupTransform("/map", "/base_link",  ros::Time(), out);
	    sendPoint(out.getOrigin().x(), out.getOrigin().y(), out.getOrigin().z() + 1.1, ros::Time());
	    
	    
	    for (int i = 0 ; i < 100 ; ++i)
		ros::spinOnce();

	    ros::Duration(0.1).sleep();
	    
	    for (int i = 0 ; i < 100 ; ++i)
		ros::spinOnce();
	}
	
    }

    void sendPoint(double x, double y, double z, const ros::Time &tm)
    {
	visualization_msgs::Marker mk;
	mk.header.frame_id = "/map";
	mk.header.stamp = tm;
	
	static int id = 0;
	mk.id = ++id;
	mk.type = visualization_msgs::Marker::SPHERE;
	mk.action = visualization_msgs::Marker::ADD;
	
	mk.pose.position.x = x;
	mk.pose.position.y = y;
	mk.pose.position.z = z;
	mk.pose.orientation.w = 1.0;
	
	mk.scale.x = mk.scale.y = mk.scale.z =  0.05;
	
	mk.color.a = 1.0;
	mk.color.r = 0.9;
	mk.color.g = 0.1;
	mk.color.b = 0.1;
	mk.lifetime = ros::Duration(10.0);
	
	visualizationMarkerPublisher_.publish(mk);
    }
    
    void waitForMove(void)
    {
	ros::Duration d(0.01);
	while(moving_)
	{
	    ros::spinOnce();
	    d.sleep();
	}
	ros::spinOnce();
    }
    
    void mm()
    {
    	cob_srvs::Trigger trig;
    	if(ros::service::call("/mm/run", trig))
    	{
    		ROS_INFO("/mm/run was called successfully");
    		return;
    	}
    	ROS_INFO("/mm/run was not called");
    }
    
protected:
    
    ros::NodeHandle node_;
    //    KDL::Tree       tree_;
    //    KDL::Chain      chain_;
   
    ros::Publisher                    visualizationMarkerPublisher_;

    boost::shared_ptr<ArmJointAction> armAction_;
    ArmJointAction::GoalHandle        armGoalHandle_;
    std::vector<std::string>          armJointName_;

    boost::shared_ptr<BaseAction>     baseAction_;
    BaseAction::GoalHandle            baseGoalHandle_;

    boost::shared_ptr<MoveAction>     moveAction_;
    MoveAction::GoalHandle            moveGoalHandle_;
    bool                              moving_;
    
    tf::TransformListener             tf_;
    
};

void script1(RobotOps &ro)
{
    ro.moveArm("folded");
    ro.waitForMove();

    ro.moveBase("home");
    ro.waitForMove();

    ro.moveBase("cob1initial0");
    ro.waitForMove();

    ro.moveBase("cob1initial");
    ro.waitForMove();

    ro.moveArm("cob1initial");
    ro.waitForMove();

    ro.move("sdh", "cylopen");
    ro.waitForMove();

    ro.moveBase("cob1pregrasp");
    ro.waitForMove();

    ro.moveBase("cob1grasp");
    ro.waitForMove();
    
    ro.move("sdh", "cylclosed");
    ro.waitForMove();
    
    ro.moveBase("cob1lbrdelivery");
    ro.waitForMove();

    ROS_INFO("Done");    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cob_my_move");
    RobotOps ro;
    //sleep(1);
    
    //if (!ro.configure())
	//return 1;
    
    ROS_INFO("lksad");
    ro.mm();

    // script1(ro);
    
    ros::spin();
    
}
