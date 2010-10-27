#include <ros/ros.h>
#include <planning_models/kinematic_model.h>
#include <planning_environment/monitors/planning_monitor.h>
#include <tf/transform_listener.h>
#include <collision_space/environmentODE.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/manifolds/RealVectorStateManifold.h>


#include <actionlib/client/action_client.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

typedef actionlib::ActionClient<pr2_controllers_msgs::JointTrajectoryAction> ArmJointAction;

class MyStateValidityChecker : public ob::StateValidityChecker
{
public:

    MyStateValidityChecker(const ob::SpaceInformationPtr &si,
			   planning_models::KinematicModel::JointGroup *jg,
			   collision_space::EnvironmentModel *em) : ob::StateValidityChecker(si), jg_(jg), em_(em)
    {
    }
    
    virtual bool isValid(const ob::State *s) const
    {
	jg_->computeTransforms(s->as<ob::RealVectorStateManifold::StateType>()->values);
	em_->updateRobotModel();
	return !em_->isCollision();
    }
    
    planning_models::KinematicModel::JointGroup *jg_;
    collision_space::EnvironmentModel           *em_;
    
};

    
class Plan
{
public:

    Plan(void)
    {
	moving_ = false;
	
	configureModels();
	configurePlanning();
	configureArm();
    }
    
    void configureModels(void)
    {
	std::string description = "/robot_description";
	
	collisionModels_ = new planning_environment::CollisionModels(description);
	planningMonitor_ = new planning_environment::PlanningMonitor(collisionModels_, &tf_);

	/*
	  The same can be done manually:

	std::string content;
	if (nh_.getParam(description, content))
	{
	    boost::shared_ptr<urdf::Model> urdf(new urdf::Model());
	    if (urdf->initString(content))
	    {

		// groups for planning; should be in a config file
		std::map< std::string, std::vector<std::string> > groups;
		groups["arm"].push_back("arm_1_joint");
		groups["arm"].push_back("arm_2_joint");
		groups["arm"].push_back("arm_3_joint");
		groups["arm"].push_back("arm_4_joint");
		groups["arm"].push_back("arm_5_joint");
		groups["arm"].push_back("arm_6_joint");
		groups["arm"].push_back("arm_7_joint"); 

		// construct kinematic model
		kmodel_ = boost::shared_ptr<planning_models::KinematicModel>(new planning_models::KinematicModel(*urdf, groups));
		kmodel_->defaultState();
		kmodel_->printModelInfo();
		
		// construct collision model
		collision_model_.reset(new collision_space::EnvironmentModelODE());
		
		std::vector<std::string> links;
		std::map<std::string, double> link_padding_map;
		collision_model_->setRobotModel(kmodel_, links, link_padding_map, 0.01, 1.0);
		collision_model_->updateRobotModel();		
	    }
	    else
	    {
		ROS_ERROR("Unable to parse URDF description!");
	    }
	}
	else
	    ROS_ERROR("Robot model '%s' not found! Did you remap 'robot_description'?", description.c_str());
	*/
    }
    
    void bricsEnv(void)
    {
	shapes::Box *table = new shapes::Box(1.4, 0.45, 0.06);
	btTransform pose;
	pose.setIdentity();
	pose.getOrigin().setX(2.7);
	pose.getOrigin().setY(1.5);
	pose.getOrigin().setZ(0.8);
	
	planningMonitor_->getEnvironmentModel()->addObject("brics", table, pose);
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

    void configurePlanning(void)
    {
	// hardcode manifold for arm
	planning_models::KinematicModel::JointGroup *g = planningMonitor_->getKinematicModel()->getGroup("arm");
	planningMonitor_->getKinematicModel()->defaultState();
	g->defaultState();
	
	ob::RealVectorStateManifold *m = new ob::RealVectorStateManifold(7);
	manifold_ = ob::StateManifoldPtr(m);
	ob::RealVectorBounds b(7);
	std::vector<double> sb = g->stateBounds;
	for (unsigned int i = 0 ; i < 7 ; ++i)
	{
	    b.low[i] = sb[2*i];
	    b.high[i] = sb[2*i + 1];
	}
	m->setBounds(b);
	ss_.reset(new og::SimpleSetup(manifold_));
	ob::StateValidityCheckerPtr svc(new MyStateValidityChecker(ss_->getSpaceInformation(), g, planningMonitor_->getEnvironmentModel()));
	ss_->setStateValidityChecker(svc);
	ss_->setup();
	ss_->print();
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
	    if (!nh_.ok())
		return false;
	    
	}
	ROS_INFO("Connected to the arm controller");
	return true;
    }

    void constructTrajectory(const og::PathGeometric &pg, trajectory_msgs::JointTrajectory &t)
    {
	if (!pg.check())
	    ROS_ERROR("Reported path is invalid");
		
	t.joint_names.resize(7);
	t.joint_names = armJointName_;

	t.points.resize(pg.states.size());
	for (unsigned int i = 0 ; i < pg.states.size() ; ++i)
	{
	    t.points[i].positions.resize(7);
	    double *vals = pg.states[i]->as<ob::RealVectorStateManifold::StateType>()->values;
	    for (unsigned int j = 0 ; j < 7 ; ++j)
		t.points[i].positions[j] = vals[j];
	    t.points[i].time_from_start = ros::Duration(3 * (i + 1));
	}
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

    void moveArm(const trajectory_msgs::JointTrajectory &trajectory)
    {
	if (trajectory.points.size() <= 4)
	{
	    moveArm4(trajectory);
	    waitForMove();
	}
	else
	{
	    unsigned int c = trajectory.points.size() / 4;
	    for (unsigned int i = 0 ; i < c ; ++c)
	    {
		trajectory_msgs::JointTrajectory tI;
		tI.joint_names = trajectory.joint_names;
		tI.points.resize(4);
		tI.points[0] = trajectory.points[i * 4];
		tI.points[1] = trajectory.points[i * 4 + 1];
		tI.points[2] = trajectory.points[i * 4 + 2];
		tI.points[3] = trajectory.points[i * 4 + 3];
		moveArm4(tI);
		waitForMove();
	    }
	    if (trajectory.points.size() % 4)
	    {
		trajectory_msgs::JointTrajectory tI;
		tI.joint_names = trajectory.joint_names;
		tI.points.resize(trajectory.points.size()  - 4 * c);
		for (unsigned int j = 0 ; j < tI.points.size() ; ++j)
		    tI.points[j] = trajectory.points[c * 4 + j];
		moveArm4(tI);
		waitForMove();
	    }
	}
    }
    
    void moveArm4(const trajectory_msgs::JointTrajectory &trajectory)
    {
	pr2_controllers_msgs::JointTrajectoryGoal goal;  
	goal.trajectory = trajectory;
	goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.2);
	
	ROS_INFO("Sending trajectory with %d points and timestamp: %f",(int)goal.trajectory.points.size(),
		 goal.trajectory.header.stamp.toSec());
	moving_ = true;
	armGoalHandle_ = armAction_->sendGoal(goal, boost::bind(&Plan::armTransitionCallback, this, _1));
    }

    void readParamTrajectory(const std::string &name, trajectory_msgs::JointTrajectory &traj)
    {
	traj.points.clear();
	
	ROS_INFO("Getting trajectory %s from param server", name.c_str());
	
	XmlRpc::XmlRpcValue v;
	if (nh_.hasParam(name))
	    nh_.getParam(name, v);
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
    
    void demoPlan(const std::string &dest)
    {
	// set the robot at the current state with all joints
	planningMonitor_->getKinematicModel()->computeTransforms(planningMonitor_->getRobotState()->getParams());
	planningMonitor_->getEnvironmentModel()->updateRobotModel();
	
	ob::ScopedState<ob::RealVectorStateManifold> start(manifold_);
	planningMonitor_->getRobotState()->copyParamsGroup(start->values, "arm");

	ob::ScopedState<ob::RealVectorStateManifold> goal(manifold_);

	// hack to read state from param server
	trajectory_msgs::JointTrajectory destT;
	readParamTrajectory(dest, destT);
	std::vector<double> state = destT.points.back().positions;
	memcpy(goal->values, &state[0], sizeof(double) * 7);
	
	// use a simple goal representation: just a state
	ss_->setStartAndGoalStates(start, goal);
	
	// figure out parameters, planner to use, etc and find a path
	if (ss_->solve())
	{
	    ss_->simplifySolution();
	    ss_->simplifySolution();
	    ss_->simplifySolution();	    
	    trajectory_msgs::JointTrajectory t;
	    constructTrajectory(ss_->getSolutionPath(), t);
	    moveArm(t);
	}
    }
    
private:

    ros::NodeHandle                        nh_;
    tf::TransformListener                  tf_;
    planning_environment::CollisionModels *collisionModels_;
    planning_environment::PlanningMonitor *planningMonitor_;


    boost::shared_ptr<ArmJointAction> armAction_;
    ArmJointAction::GoalHandle        armGoalHandle_;
    std::vector<std::string>          armJointName_;
    bool                              moving_;
    
    
    ob::StateManifoldPtr manifold_;
    boost::shared_ptr<og::SimpleSetup> ss_;
    
};

    
int main(int argc, char **argv)
{
    ros::init(argc, argv, "cob_my_plan");
    sleep(1);
    
    Plan p;

    
    p.demoPlan("/script_server/arm/overtablet");
    //    p.demoPlan("/script_server/arm/folded");
    
    
    ros::spin();
    
}
