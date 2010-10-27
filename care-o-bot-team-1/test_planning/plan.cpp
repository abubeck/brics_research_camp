#include <ros/ros.h>
#include <planning_models/kinematic_model.h>
#include <collision_space/environmentODE.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/manifolds/RealVectorStateManifold.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class MyStateValidityChecker : public ob::StateValidityChecker
{
public:

    MyStateValidityChecker(const ob::SpaceInformationPtr &si,
			   planning_models::KinematicModel::JointGroup *g,
			   collision_space::EnvironmentModel *e) : ob::StateValidityChecker(si)
    {
    }
    
    virtual bool isValid(const ob::State *s) const
    {
	jg->computeTransforms(s->as<ob::RealVectorStateManifold::StateType>()->values);
	em->updateRobotModel();
	return !em->isCollision();
    }
    
    planning_models::KinematicModel::JointGroup *jg;
    collision_space::EnvironmentModel *em;
    
};

    
class Plan
{
public:

    Plan(void)
    {
    }
    
    void configure(void)
    {
	std::string description = "/robot_description";
	
	std::string content;
	if (nh_.getParam(description, content))
	{
	    boost::shared_ptr<urdf::Model> urdf(new urdf::Model());
	    if (urdf->initString(content))
	    {
		std::map< std::string, std::vector<std::string> > groups;
		groups["arm"].push_back("arm_1_joint");
		groups["arm"].push_back("arm_2_joint");
		groups["arm"].push_back("arm_3_joint");
		groups["arm"].push_back("arm_4_joint");
		groups["arm"].push_back("arm_5_joint");
		groups["arm"].push_back("arm_6_joint");
		groups["arm"].push_back("arm_7_joint"); 
		kmodel_ = boost::shared_ptr<planning_models::KinematicModel>(new planning_models::KinematicModel(*urdf, groups));
		kmodel_->defaultState();
		kmodel_->printModelInfo();

		collision_model_.reset(new collision_space::EnvironmentModelODE());
		
		std::vector<std::string> links;
		std::map<std::string, double> link_padding_map;
		collision_model_->setRobotModel(kmodel_, links, link_padding_map, 0.01, 1.0);
		collision_model_->updateRobotModel();
		
		
		ob::RealVectorStateManifold *m = new ob::RealVectorStateManifold(7);
		manifold_ = ob::StateManifoldPtr(m);
		ob::RealVectorBounds b(7);
		std::vector<double> sb = kmodel_->getGroup("arm")->stateBounds;
		for (unsigned int i = 0 ; i < 7 ; ++i)
		{
		    b.low[i] = sb[2*i];
		    b.high[i] = sb[2*i + 1];
		}
		m->setBounds(b);
		ss_.reset(new og::SimpleSetup(manifold_));
		ss_->setStateValidityChecker(ob::StateValidityCheckerPtr(new MyStateValidityChecker(ss_->getSpaceInformation(), kmodel_->getGroup("arm"), collision_model_.get())));
		
	    }
	    else
	    {
		ROS_ERROR("Unable to parse URDF description!");
	    }
	}
	else
	    ROS_ERROR("Robot model '%s' not found! Did you remap 'robot_description'?", description.c_str());
    }
    
    void demoPlan(void)
    {
	ob::ScopedState<> start(manifold_);
	start.random();
	ob::ScopedState<> goal(manifold_);
	goal.random();
	
	ss_->setStartAndGoalStates(start, goal);
	ss_->solve();
    }
    
private:

    ros::NodeHandle nh_;
    boost::shared_ptr<planning_models::KinematicModel> kmodel_;
    boost::shared_ptr<collision_space::EnvironmentModel> collision_model_;
    
    ob::StateManifoldPtr manifold_;
    boost::shared_ptr<og::SimpleSetup> ss_;
    
};

    
int main(int argc, char **argv)
{
    ros::init(argc, argv, "cob_my_plan");
    sleep(1);
    
    Plan p;
    p.configure();

    p.demoPlan();
    
    
    ros::spin();
    
}
