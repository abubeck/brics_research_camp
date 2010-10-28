#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <planning_environment/monitors/planning_monitor.h>
#include <actionlib/client/action_client.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/manifolds/RealVectorStateManifold.h>
// #include <ompl/geometric/planners/prm/PRM.h>

#include <test_planning/ComputeArmPlan.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;


/** \brief The state validity checking routine for the care-o-bot arm in the format expected by OMPL */
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

private:
    
    planning_models::KinematicModel::JointGroup *jg_;
    collision_space::EnvironmentModel           *em_;
};


/** Simple encapsulation of motion planning OMPL with collision detection using planning_environment */
class ArmPlan
{
public:

    ArmPlan(void) : nh_("~")
    {
	configureModels();
	configurePlanning();
	//	bricsEnv();

	connectService();
    }
    
    void connectService(void)
    {
	planService_ = nh_.advertiseService("plan_arm_path", &ArmPlan::planToGoal, this);
    }
    
    bool planToGoal(test_planning::ComputeArmPlan::Request &req, test_planning::ComputeArmPlan::Response &res)
    {
	if (req.goal.position.size() != 7)
	{
	    ROS_ERROR("Expected 7 joint values for the arm");
	    return false;
	}
	
	ob::ScopedState<ob::RealVectorStateManifold> goal(manifold_);
	for (unsigned int i = 0 ; i < 7 ; ++i)
	    goal->values[i] = req.goal.position[i];
	planTo(goal, res.path);
	
	return res.path.points.size() > 0;
    }
    
    /** \brief Create representation of the robot kinematic tree using
	planning_models and an environment representation using
	collision_space. This is automatically managed by
	planning_environment.  Param server needs to be configured a
	priori. Collision information is automatically incorporated as
	well, if available. */
    void configureModels(void)
    {
	std::string description = "robot_description";
	
	collisionModels_ = new planning_environment::CollisionModels(description);
	planningMonitor_ = new planning_environment::PlanningMonitor(collisionModels_, &tf_);
    }
    
    /** \brief Set up a fake environment. This should be replaced by perception. */
    void bricsEnv(void)
    {
	shapes::Box *table = new shapes::Box(1.4, 0.45, 0.06);
	btTransform pose;
	pose.setIdentity();
	pose.getOrigin().setX(1.2);
	pose.getOrigin().setY(0);
	pose.getOrigin().setZ(0.8);
	
	planningMonitor_->getEnvironmentModel()->addObject("brics", table, pose);
    }
    
    /// Construct the state space for the arm and pass it to ompl
    void configurePlanning(void)
    {
	// get the group of joints that corresponds to the arm
	planning_models::KinematicModel::JointGroup *g = planningMonitor_->getKinematicModel()->getGroup("arm");
	
	// create a R^7 manifold
	ob::RealVectorStateManifold *m = new ob::RealVectorStateManifold(7);
	manifold_ = ob::StateManifoldPtr(m);

	// set the bounds for the manifold
	ob::RealVectorBounds b(7);
	std::vector<double> sb = g->stateBounds;
	for (unsigned int i = 0 ; i < 7 ; ++i)
	{
	    b.low[i] = sb[2*i];
	    b.high[i] = sb[2*i + 1];
	}
	m->setBounds(b);

	// create a default OMPL setup
	ss_.reset(new og::SimpleSetup(manifold_));

	// set state validity checking 
	ob::StateValidityCheckerPtr svc(new MyStateValidityChecker(ss_->getSpaceInformation(), g, planningMonitor_->getEnvironmentModel()));
	ss_->setStateValidityChecker(svc);

	// printing information 
	ss_->print();
    }

    /// convert a path produced by OMPL to a path that can be sent to the controller
    void constructTrajectory(og::PathGeometric &pg, trajectory_msgs::JointTrajectory &t)
    {
	if (!pg.check())
	    ROS_ERROR("Reported path is invalid");
	
	/// make sure the path has more points than needed, so that linear interpolation need not direclty be used by the arm
	pg.interpolate(16);
	
	t.joint_names.resize(7);
	t.joint_names = planningMonitor_->getKinematicModel()->getGroup("arm")->jointNames;
	
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

    /// plan to a destination read from the param server
    void planTo(const ob::ScopedState<> &goal, trajectory_msgs::JointTrajectory &t)
    {
	// set the robot at the current state with all joints
	planningMonitor_->getKinematicModel()->computeTransforms(planningMonitor_->getRobotState()->getParams());
	planningMonitor_->getEnvironmentModel()->updateRobotModel();
	
	ob::ScopedState<ob::RealVectorStateManifold> start(manifold_);
	planningMonitor_->getRobotState()->copyParamsGroup(start->values, "arm");
	
	// use a simple goal representation: just a state
	ss_->setStartAndGoalStates(start, goal);

	// figure out parameters, planner to use, etc and find a path
	if (ss_->solve())
	{
	    ss_->simplifySolution();
	    ss_->simplifySolution();
	    ss_->simplifySolution();	    

	    constructTrajectory(ss_->getSolutionPath(), t);
	}
	else
	    ROS_WARN("Solution not found");
    }
    
private:

    ros::NodeHandle                        nh_;
    tf::TransformListener                  tf_;
    planning_environment::CollisionModels *collisionModels_;
    planning_environment::PlanningMonitor *planningMonitor_;

    ros::ServiceServer                     planService_;
    
    ob::StateManifoldPtr                   manifold_;
    boost::shared_ptr<og::SimpleSetup>     ss_;
    
};

    
int main(int argc, char **argv)
{
    ros::init(argc, argv, "cob_arm_planning");
    sleep(1);
    
    ArmPlan p;
    ros::spin();
    
    return 0;
}
