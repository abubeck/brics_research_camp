#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>

class GetState
{

public:
    GetState(void)
    {
	js_ = node_.subscribe("/joint_states", 1, &GetState::controllerStateCallback, this);
    }
    
    std::vector<double> parseJointStates(std::vector<std::string> names, std::vector<double> positions)
    {
	std::vector<double> q_temp(7);
	int count = 0;
	for(unsigned int i = 0; i < names.size(); i++)
	{
	    if(strncmp(names[i].c_str(), "arm_", 4) == 0)
	    {
		q_temp[count] = positions[i];
		count++;
	    }
	}
	if (count == 0)
	    return std::vector<double>();
	else
	    return q_temp;
    }
    
    void controllerStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
	std::vector<std::string> names = msg->name;
	std::vector<double> positions = msg->position;
	std::vector<double> q = parseJointStates(names, positions);
	if (q.empty())
	    return;
	
	static int x = 0;
	x++;
	if (x % 10 == 1)
	{
	    tf::StampedTransform out;
	    tf_.lookupTransform("/map", "/base_link",  ros::Time(), out);
	    double yaw = tf::getYaw(out.getRotation());
	    std::cout << "x, y, yaw = " << out.getOrigin().x() << " " << out.getOrigin().y() << " " << yaw << std::endl;
	    for (unsigned int i = 0 ; i < 7 ; ++i)
		std::cout << q[i] << " ";
	    std::cout << std::endl;
	}
    }
    
    
    ros::NodeHandle       node_;
    ros::Subscriber       js_;
    tf::TransformListener tf_;
};

    
int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_positions");
    GetState gs;
    sleep(1);
    
    ros::spin();
}
