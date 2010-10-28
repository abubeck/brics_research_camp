/**
 * @file 
 * arc_detection.cpp
 *
 * @date: Oct 1, 2010
 * @author: sblume
 */

#include <iostream>
#include <string>
using namespace std;

#include <cstdlib>
#include <cstring>
#include <cmath>
#include <limits>


#include "feature_geometry.hpp"
#include "feature_person.hpp"

/*ROS includes*/
#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "tf/transform_broadcaster.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"


#include <dynamic_reconfigure/server.h>
#include <iav_legtrack/IAVLegTrackConfig.h>


class ArcDetector {

private:
	ros::NodeHandle n_;
	ros::Subscriber sub;
	ros::Publisher scan_pub_;

	tf::TransformBroadcaster br;
	tf::Transform transform;


	laser_geometry::LaserProjection projector_;
//	LaserFeatureX laserFeature; //IAV algorithm handle
	std::string laserFrameID;

public:
	LaserFeatureX laserFeature; //IAV algorithm handle
ArcDetector(ros::NodeHandle n) :
		n_(n)
	{
		/* setup input/output communication */
		sub = n_.subscribe("scan", 1000, &ArcDetector::scannerCallback, this);
		scan_pub_ = n_.advertise<sensor_msgs::PointCloud>("/my_cloud",1);

		laserFrameID = "laser";

		/* set default configuration values */
		iav_legtrack::IAVLegTrackConfig dafultConfig;
		callback(dafultConfig, 0);

	}

void scannerCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
	detectArcs(scan_in);
}

void detectArcs (const sensor_msgs::LaserScan::ConstPtr& scan_in) {

	ROS_INFO("hallo world");

	/* create point clouds */
	sensor_msgs::PointCloud cloud; //same as scan, but in Cartesian coordinates
	projector_.projectLaser(*scan_in, cloud); //version without tf

	sensor_msgs::PointCloud featureCloud; // contains the centers of the circle features
	geometry_msgs::Point32 tmpPoint;
	featureCloud.points.push_back(tmpPoint); // Add artificial (0,0,0) (only for visualization)
	featureCloud.header = scan_in->header;

	bool check2legs; // This name comes from Joao's pv3d code.

	/* feed data to IVA structures */
	laserFeature.fdata.clear();
	laserFeature.ranges.clear();
	laserFeature.point_xy.clear();
	for (uint i = 0; i < scan_in->ranges.size(); i++) {
		laserFeature.ranges.push_back( scan_in->ranges[i]);
		laserFeature.point_xy.push_back( xy( cloud.points[i].x, cloud.points[i].y ) );
	}

	/* segment scan according to proximity */
	laserFeature.Segmentation();
	
	int nearestDistanceFeatureIndex;
	float segmentLength;
	float currentDistance, nearestAngle, nearestDistance;
	float nearestPositionX, nearestPositionY, meansegX, meansegY;
	currentDistance = nearestAngle =nearestPositionX=nearestPositionY=meansegX=meansegY=segmentLength=0;
	nearestDistance = 999;
	nearestDistanceFeatureIndex = 0;


	/* check segments for lines, arcs and legs */
	for (uint i = 0; i < laserFeature.segments.size(); i++) {
		
	//	printf("currentX %d : %f, currentY %d : %f\n", i, laserFeature.segbeginx[i], i, laserFeature.segbeginy[i]);
		meansegX = (laserFeature.segbeginx[i]+laserFeature.segendx[i])/2;
		meansegY = (laserFeature.segbeginy[i]+laserFeature.segendy[i])/2;
		segmentLength = sqrt((laserFeature.segbeginx[i]*laserFeature.segendx[i]) + (laserFeature.segbeginy[i]+laserFeature.segendy[i]));
		printf("%d  meansegX : %f, meansegY : %f, segSize : %f\n", i, meansegX, meansegY, segmentLength);

		//// detect the nearest object ////
		currentDistance=sqrt(meansegX*meansegX+meansegY*meansegY);
		if (currentDistance < nearestDistance && currentDistance>0 && segmentLength < 2.0) {
				nearestDistance = currentDistance;
				nearestPositionX = meansegX;
				nearestPositionY = meansegY;
				nearestDistanceFeatureIndex = i;
			}

	}
	



	tmpPoint.x = nearestPositionX;
	tmpPoint.y = nearestPositionY;
	tmpPoint.z = 0;
	featureCloud.points.push_back(tmpPoint);

	printf("nearX : %f, nearY : %f\n", tmpPoint.x, tmpPoint.y);
	printf("nearestDistance = %f, index = %d\n", nearestDistance, nearestDistanceFeatureIndex);

	transform.setOrigin( tf::Vector3(tmpPoint.x, tmpPoint.y, 0.0) );
	transform.setRotation( tf::Quaternion(0, 0, 0) );
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), laserFrameID, "nearest_arc"));

	/* publish nearest arc as frame */


	/* publish all detected arcs as point cloud */
	scan_pub_.publish(featureCloud);
	

}

void callback(iav_legtrack::IAVLegTrackConfig &config, uint32_t level)
{

	  ROS_INFO("Reconfigure request : %f %f %f %f %f %f %f %f %f %i %i %i %i %i",
			  config.max_laser_range,
			  config.arc_min_aperture,
			  config.arc_max_aperture,
			  config.arc_std_max,
			  config.segmentation_threshold,
			  config.line_min_distance,
			  config.line_error_threshold,
			  config.max_leg_diameter,
			  config.min_leg_diameter,
            (int) config.do_circles,
            (int) config.do_lines,
            (int) config.do_legs,
            (int) config.iav_do_lines,
            (int) config.safe_circle_corners);

	  //set the new config
		laserFeature.max_laser_range = config.max_laser_range;
		laserFeature.arc_min_aperture = config.arc_min_aperture;
		laserFeature.arc_max_aperture = config.arc_max_aperture;
		laserFeature.arc_std_max = config.arc_std_max;
		laserFeature.segmentation_threshold = config.segmentation_threshold;
		laserFeature.line_min_distance = config.line_min_distance;
		laserFeature.line_error_threshold = config.line_error_threshold;
		laserFeature.max_leg_diameter = config.max_leg_diameter;
		laserFeature.min_leg_diameter = config.min_leg_diameter;
		laserFeature.do_circles = config.do_circles;
		laserFeature.do_lines = config.do_lines;
		laserFeature.do_legs = config.do_legs;
		laserFeature.iav_do_lines = config.iav_do_lines;
		laserFeature.safe_circle_corners = config.safe_circle_corners;

}


}; // end of class

int main(int argc, char** argv)
{

	ros::init(argc, argv, "arc_detection");
	ros::NodeHandle n;
	ArcDetector arcDetector(n);

	dynamic_reconfigure::Server<iav_legtrack::IAVLegTrackConfig> srv;
	dynamic_reconfigure::Server<iav_legtrack::IAVLegTrackConfig>::CallbackType f = boost::bind(&ArcDetector::callback, &arcDetector, _1, _2);
	srv.setCallback(f);


	ros::spin();
	return 0;
}





/* EOF */
