#ifndef _LASERFEATUREX_
#define _LASERFEATUREX_

/**
@par Configuration file options
- laser_index (integer)
- default: 0
- index of laser device to use

- do_circles (integer)
- default: 1
- algorithm to extract circles, 0 for none

- do_lines (integer)
- default: 1
- algorithm to extract lines, 0 for none

- do_legs (integer)
- default: 1
- algorithm to extract legs, 0 for none

- line_error_threshold (integer)
- default: 5
- maximum error in mm to split line in 2

- arc_min_aperture (float)
- default: 1.57
- minimum aperture of the arc in radians

- arc_max_aperture
- default: 2.5 (float)
- minimum aperture of the arc in radians

- max_leg_diameter (integer)
- default: 200
- maximum leg diameter in mm

- min_leg_diameter
- default: 40
- minimum leg diameter in mm

- line_min_distance (integer)
- default: 170
- minimum length of a line

- segmentation_threshold (integer)
- default: 120
- the distance threshold to create new segments

- arc_std_max (float)
- default: 0.20
- maximum standard deviation to classify arcs
@par Example

@verbatim
laser:0 ( driver "passthrough" port 6665 index 0 )
position:0 ( driver "passthrough" port 6665 index 0 )

device
(
plugin "lasergeometry.so"
driver "lasergeometry"
interfaces ["feature:0"]
laser_index 0
do_circles 1
do_lines 1
do_legs 1
line_error_threshold 5
arc_min_aperture 1.57 #90 degrees
arc_max_aperture 2.5 #2.365 #135 degrees
max_leg_diameter 200 #200 #180 #240 #200
min_leg_diameter 40 #60

line_min_distance 170
segmentation_threshold 120
arc_std_max 0.20 #0.15
)
@endverbatim

@par Authors
Joao Xavier smogzer_at_gmail.com
*/
/** @} */

#include <cmath>
#include <vector>

#include "fiducial.hpp"
#include "xy.hpp"

typedef unsigned int uint;

struct LaserFeatureX {

	LaserFeatureX();

    struct segment {
		segment(int in_begin=0, int in_end=0) : begin(in_begin),end(in_end) {}
        int begin;
        int end;
    };

    std::vector<float> ranges;
    std::vector<segment> segments;
    std::vector<xy> point_xy;

    miarn_fiducial_data fdata;

    /// Configuration options
	float max_laser_range;
    float arc_min_aperture;
    float arc_max_aperture;
    float arc_std_max;
    float segmentation_threshold;
    float line_min_distance;
    float line_error_threshold;
    float max_leg_diameter;
    float min_leg_diameter;
    bool do_circles,do_lines,do_legs,iav_do_lines;
    bool safe_circle_corners;

    /// functions
    float Average(float a[],int size_a);
    float StandardDeviation(float a[],int size_a,float meanvalue);
    bool FitArc(int begin, int end);
    void Segmentation();
    bool CirclePreRequisites(const int begin,const int end);
    bool LinePreRequisites(const int& begin,const int& end);
    bool FindLeg(const int begin,const int end, bool check2legs);
    void lineDef(float &A,float &B,float &C, const int begin,const int end);
    void LinearRegression (float &m,float &b, float &r, int begin, int end);
    void RecursiveLineFitting(int begin, int end);

    void AddProbableTwoLegs(float pos_x, float pos_y,
							float leg1_x,float leg1_y,
							float leg2_x,float leg2_y,
							float begin_x, float end_x, float  begin_y, float end_y,
							int begin_index, int end_index);
    void AddProbableLeg(float pos_x, float pos_y,
						float begin_x, float end_x, float  begin_y, float end_y,
						int begin_index, int end_index);
    void AddLine(float slope, float offset,float maxerror,
				 float begin_x, float end_x, float  begin_y, float end_y,
				 int begin_index, int end_index);
    void AddArc(float pos_x, float pos_y, float radius,
				float begin_x, float end_x, float  begin_y, float end_y,
				int begin_index, int end_index,float average, float std_dev);
    void AddSegment(float begin_x, float end_x, float  begin_y, float end_y,
					int begin_index, int end_index);
};

#endif
