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
	LaserFeatureX laserFeature; //IAV algorithm handle
	std::string laserFrameID;

public:

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
//		laserFeature.max_laser_range = 8; //(8),
//		laserFeature.arc_min_aperture = 1.5;//(1.57),
//		laserFeature.arc_max_aperture = 3.5; //(2.365), //<-huge impact
//		laserFeature.arc_std_max = 0.25; //(0.25),
//		laserFeature.segmentation_threshold = 0.200; //(.200),
//		laserFeature.line_min_distance = 0.120; //(.120),
//		laserFeature.line_error_threshold = 0.020; //(.020),
//		laserFeature.max_leg_diameter = 0.200; //(.200),
//		laserFeature.min_leg_diameter = 0.040; //(.040),
//		laserFeature.do_circles = true; //(true),
//		laserFeature.do_lines = true; //(true),
//		laserFeature.do_legs = true; //(true),
//		laserFeature.iav_do_lines = true; //(true),
//		laserFeature.safe_circle_corners = false; //(false)
	}

void scannerCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
	detectArcs(scan_in);
}

void detectArcs (const sensor_msgs::LaserScan::ConstPtr& scan_in) {

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

	/* check segments for lines, arcs and legs */
	for (uint i = 0; i < laserFeature.segments.size(); i++) {

		// Fit lines
		laserFeature.RecursiveLineFitting( laserFeature.segments[i].begin, laserFeature.segments[i].end );

		// Fit circles
		check2legs = !laserFeature.FitArc( laserFeature.segments[i].begin, laserFeature.segments[i].end );

		// Find legs
		laserFeature.FindLeg( laserFeature.segments[i].begin, laserFeature.segments[i].end, check2legs );
	}

	/* print all features found */
	int numCircles, numLines, numPLeg, numP2Legs;
	numCircles = numLines = numPLeg = numP2Legs = 0;
	for (uint i = 0; i < laserFeature.fdata.size(); i++) {
		switch (laserFeature.fdata[i].type) {
		case MIARN_FEATURE_TYPE_ARC:
			numCircles++;
			break;
		case MIARN_FEATURE_TYPE_LINE:
			numLines++;
			break;
		case MIARN_FEATURE_TYPE_PLEG:
			numPLeg++;
			break;
		case MIARN_FEATURE_TYPE_2LEGS:
			numP2Legs++;
			break;
		case MIARN_FEATURE_TYPE_SEGMENT:
			// Ignore these.
			break;
		default:
			cerr << "Error: unrecognized feature type code: " << laserFeature.fdata[i].type << endl;
		}
	}
	if (numCircles > 0 || numLines > 0 || numPLeg > 0 || numP2Legs > 0)
		cout << "Found on this cycle"
		<< "\n  circles: " << numCircles
		<< "\n  lines: " << numLines
		<< "\n  pleg: " << numPLeg
		<< "\n  p2legs: " << numP2Legs << endl;

	/* go through all arcs (because here we are only interested in arcs) and take nearest one */
	int nearestArcFeatureIndex = -1;
	float nearestArcDistance = numeric_limits<float>::max();
	int currentArcScanMidIndex = 0; //here we take the range of beam that belongs to the "middle" of the arc segment, rather than the center of the calculated arc. This is more accurate.
	float currentArcDistance = 0.0;
	for (uint i = 0; i < laserFeature.fdata.size(); i++) {
		switch (laserFeature.fdata[i].type) {
		case MIARN_FEATURE_TYPE_ARC:

			/* print info */
			cout << "Arc found at center of ( " << laserFeature.fdata[i].pos[0] << ", " //TODO
			<< laserFeature.fdata[i].pos[1] << " ), at angle "
			<< atan2f( laserFeature.fdata[i].pos[1], laserFeature.fdata[i].pos[0] )*180/3.14 <<
			" deg, with radius " << laserFeature.fdata[i].extra[2] << endl;

			/* add to output point cloud */
			tmpPoint.x = laserFeature.fdata[i].pos[0];
			tmpPoint.y = laserFeature.fdata[i].pos[1];
			tmpPoint.z = 0;
			featureCloud.points.push_back(tmpPoint);

			/* hook code in here to publish several frames, poses, etc. */

			/* memorize "nearest" arc */
			currentArcScanMidIndex = (laserFeature.fdata[i].end_index) - (laserFeature.fdata[i].begin_index);
			currentArcDistance = laserFeature.ranges[currentArcScanMidIndex];
			if (currentArcDistance < nearestArcDistance) {
				nearestArcDistance = currentArcDistance;
				nearestArcFeatureIndex = i;
			}

			break;
		case MIARN_FEATURE_TYPE_LINE:
		case MIARN_FEATURE_TYPE_PLEG:
		case MIARN_FEATURE_TYPE_2LEGS:
		case MIARN_FEATURE_TYPE_SEGMENT:
			// Ignore these.
			break;
		default:
			cerr << "Error: unrecognized feature type code: " << laserFeature.fdata[i].type << endl;
		}
	}

	ROS_INFO("nearestArcFeatureIndex: %i of %i features \n", nearestArcFeatureIndex, laserFeature.fdata.size());
	if (nearestArcFeatureIndex >= 0) { //only if an arc is found
		//laserFeature.fdata.[nearestArcIndex] (laserFeature.fdata[nearestArcIndex].end_index) - (laserFeature.fdata[nearestArcIndex].begin_index)
		//cloud.points[nearestArcIndex].x
		currentArcScanMidIndex = (laserFeature.fdata[nearestArcFeatureIndex].end_index) - (laserFeature.fdata[nearestArcFeatureIndex].begin_index);
		//transform.setOrigin( tf::Vector3(cloud.points[currentArcScanMidIndex].x, cloud.points[currentArcScanMidIndex].y, 0.0) );
		transform.setOrigin( tf::Vector3(laserFeature.fdata[nearestArcFeatureIndex].pos[0], laserFeature.fdata[nearestArcFeatureIndex].pos[1], 0.0) );
		transform.setRotation( tf::Quaternion(0, 0, 0) ); //TODO calculate tangent
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), laserFrameID, "nearest_arc"));

	} else {
		//TODO
	}

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
