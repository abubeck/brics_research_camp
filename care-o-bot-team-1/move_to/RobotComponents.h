#include <vector>
#include <string>

/** \brief List of robot components */
class RobotComponents
{
public:
    
    /** \brief Structure containing arm information */
    struct ArmInformation
    {
	std::vector<std::string> jointNames;
	
	std::string trajectoryAction;
	
	/// name of service to get info about joints IK service knows about
	std::string ik_info_service_name;
	
	/// name of IK service
	std::string ik_service_name;
    };
    
    /** \brief The group names identifying arms */
    std::vector<ArmInformation> arms;
    
    /** \brief The group name identifying the mobile base */
    std::string base;
    
    std::string baseAction;
    
    // global fixed frame
    std::string mapFrame;
    
    /** \brief The frame is assumed to move with the robot but
	stay fixed while moving the arms. It is also assumed the
	environment is specified with respect to this frame. */
    std::string robotFrame;	
};

static inline RobotComponents cob_info(void)
{
    RobotComponents rc;
    rc.mapFrame = "map";
    rc.robotFrame = "base";
    rc.baseController = "";
    
    rc.arms.resize(1);
    rc.arms[0].trajectoryAction = "arm_controller/joint_trajectory_action";
    
    return rc;
}
