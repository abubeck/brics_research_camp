/** Developed at BRICS 2010 */

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <sstream>    
#include <fstream>

/** \brief Record a sequence of joint states and base positions in
    order to be able to replay motions */
class RecordSequence
{
public:
    
    RecordSequence(void) :  node_("~")
    {
	readParameters();
	js_ = node_.subscribe(jointStatesTopic_, 1, &RecordSequence::jointStateCallback, this);
    }
    
    /** \brief Read parameters from the param server */
    void readParameters(void)
    {
	node_.param<std::string>("joint_state_topic", jointStatesTopic_, "joint_states");
	
	if (node_.hasParam("recorded_joints"))
	{
	    XmlRpc::XmlRpcValue v;
	    node_.getParam("recorded_joints", v);
	    for (int i = 0 ; i < v.size() ; ++i)
		jointNames_.push_back((std::string)v[i]);
	}
	
	for (int dim = 2 ; dim <= 3 ; ++dim)
	{
	    std::vector< std::pair<std::string, std::string> > &frameTransforms = 
		dim == 2 ? frameTransforms2D_ : frameTransforms3D_;
	    std::string paramName = dim == 2 ? "frame_transforms2D" : "frame_transforms3D";
	    
	    if (node_.hasParam(paramName))
	    {
		XmlRpc::XmlRpcValue v;
		node_.getParam(paramName, v);
		for (int i = 0 ; i < v.size() ; ++i)
		{
		    std::string framePair = (std::string)v[i];
		    std::size_t pos = framePair.find_first_of("-");
		    if (pos == std::string::npos)
			ROS_WARN("Frame transforms need to be provided in the format 'frame1-frame2'");
		    else
			frameTransforms.push_back(std::make_pair(framePair.substr(0, pos), framePair.substr(pos + 1)));
		}
	    }
	}
	
	ROS_INFO("Reading joint values from %s topic", jointStatesTopic_.c_str());
	ROS_INFO("Recording joint values for");
	for (unsigned int i = 0 ; i < jointNames_.size() ; ++i)
	    ROS_INFO("   %s", jointNames_[i].c_str());
	ROS_INFO("Recording frame 2D transforms pairs for");
	for (unsigned int i = 0 ; i < frameTransforms2D_.size() ; ++i)
	    ROS_INFO("   %s - %s", frameTransforms2D_[i].first.c_str(), frameTransforms2D_[i].second.c_str());	
	ROS_INFO("Recording frame 3D transforms pairs for");
	for (unsigned int i = 0 ; i < frameTransforms3D_.size() ; ++i)
	    ROS_INFO("   %s - %s", frameTransforms3D_[i].first.c_str(), frameTransforms3D_[i].second.c_str());	
    }

    /** \brief Callback for receiving joint values */
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
	for (unsigned int i = 0 ; i < msg->name.size() ; ++i)
	    lastSeenValues_[msg->name[i]] = msg->position[i];
    }
    
    /** \brief This function prints the information to be recorded to
	a stream of choice, under a specified name */
    void printCurrent(const std::string &recName, std::ostream &out)
    {
	out << recName << "joints: [";
	for (unsigned int i = 0 ; i < jointNames_.size() ; ++i)
	{
	    if (lastSeenValues_.find(jointNames_[i]) == lastSeenValues_.end())
		ROS_WARN_ONCE("Joint %s was not found", jointNames_[i].c_str());
	    out << lastSeenValues_[jointNames_[i]];
	    if (i + 1 < jointNames_.size())
		out << ", ";
	}
	out << "]" << std::endl;
	
	for (unsigned int i = 0 ; i < frameTransforms2D_.size() ; ++i)
	{
	    tf::StampedTransform t;
	    try
	    {
		tf_.lookupTransform(frameTransforms2D_[i].first, frameTransforms2D_[i].second, ros::Time(), t);
		out << recName << "base2D: [" << t.getOrigin().x() << ", " << t.getOrigin().y() <<
		    ", " << tf::getYaw(t.getRotation()) << "]" << std::endl;
	    }
	    catch(...)
	    {
		// tf may have not received data yet; not a big problem
	    }
	}
	
	for (unsigned int i = 0 ; i < frameTransforms3D_.size() ; ++i)
	{
	    tf::StampedTransform t;
	    try
	    {
		tf_.lookupTransform(frameTransforms3D_[i].first, frameTransforms3D_[i].second, ros::Time(), t);
		out << recName << "base3D: [" << t.getOrigin().x() << ", " << t.getOrigin().y() << ", " << t.getOrigin().z()
		    << ", " << t.getRotation().x() <<  ", " << t.getRotation().y() << ", " << t.getRotation().z() << ", " 
		    << t.getRotation().w() << "]" << std::endl;
	    }
	    catch(...)
	    {
		// tf may have not received data yet; not a big problem
	    }	    
	}
    }
    
private:
    
    /** \brief */
    std::string                                        jointStatesTopic_;
    std::vector<std::string>                           jointNames_;
    std::map<std::string, double>                      lastSeenValues_;
    std::vector< std::pair<std::string, std::string> > frameTransforms2D_;
    std::vector< std::pair<std::string, std::string> > frameTransforms3D_;

    ros::NodeHandle                                    node_;
    ros::Subscriber                                    js_;
    tf::TransformListener                              tf_;
};

void spinThread(void)
{
    ros::spin();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "record_sequence", ros::init_options::AnonymousName);
    sleep(1);
    
    RecordSequence rec;
    boost::thread thr(&spinThread);

    std::stringstream sequence;
    std::stringstream order;
    
    bool done = false;    
    while (!done && std::cin.good() && !std::cin.eof())
    {
	std::cout << "Type 'v' for viewing the current state," << std::endl;
	std::cout << "     'r [name]' to record a state," << std::endl;
	std::cout << "     'x' to exit." << std::endl;
	std::cout << "> ";
	char buffer[256];
	std::cin.getline(buffer, sizeof(buffer));
	std::string command = buffer;
	if (!command.empty())
	{
	    switch(command[0])
	    {
	    case 'r':
	    case 'R':
		{
		    std::cout << command << std::endl;
		    
		    std::string name = command.substr(1);
		    std::cout << name << std::endl;
		    while (!name.empty() && name[0] == ' ')
		    {
			name = name.substr(1); std::cout << name << std::endl;
		       
		    }
		    
		    if (name.empty())
		    {
			static int count = 0;
			std::stringstream number;
			number << (++count);
			name = "pose" + number.str();
		    }
		    rec.printCurrent(name, sequence);
		    order << name << " ";
		    rec.printCurrent(name, std::cout);
		}
	    break;
	    case 'x':
	    case 'X':
		done = true;
	    break;
	    case 'v':
	    case 'V':
	    default:
		rec.printCurrent("current", std::cout);
		break;
	    }
	}
    }
    if (!sequence.str().empty())
    {
	if (argc > 1)
	{
	    std::ofstream fout(argv[1]);
	    fout << std::endl << sequence.str() << std::endl;
	    fout << "order: [ " << order.str() << "]" << std::endl << std::endl;
	}
	else
	{
	    std::cout << std::endl << sequence.str() << std::endl;
	    std::cout << "order: [ " << order.str() << "]" << std::endl << std::endl;
	}
    }
    thr.interrupt();
    thr.join();
    
    return 0;
}
