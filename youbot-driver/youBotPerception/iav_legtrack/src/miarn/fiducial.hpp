#ifndef _MIARN_FIDUCIAL_LASER_TYPES_
#define _MIARN_FIDUCIAL_LASER_TYPES_

#define MIARN_FEATURE_TYPE_ARC 1
#define MIARN_FEATURE_TYPE_LINE 6
#define MIARN_FEATURE_TYPE_PLEG 7
#define MIARN_FEATURE_TYPE_2LEGS 8
#define MIARN_FEATURE_TYPE_LEG 10
#define MIARN_FEATURE_TYPE_PERSON 11
#define MIARN_FEATURE_TYPE_SEGMENT 20

#include <strings.h> // for bzero

typedef unsigned int uint;

/// The fiducial data packet contains a list of these.
struct miarn_fiducial_item {
miarn_fiducial_item (){bzero(this,sizeof(*this));}
    int type;
///   The fiducial id.  Fiducials that cannot be identified get id -1.
    int id;
    /** Fiducial position relative to the detector (x, y, z) in meters. */
    float pos[3];
    /** Fiducial orientation relative to the detector (r, p, y) in millirad. */
    float rot[3];
    /** Uncertainty in the measured pose (x, y, z) in meters. */
    float upos[3];
    /** Uncertainty in fiducial orientation relative to the detector
        (r, p, y) in millirad. */
    float urot[3];
    int begin_index,end_index;
    float begin_x,end_x;
    float begin_y,end_y;
    float extra[4];
};

#include <vector>

typedef std::vector<miarn_fiducial_item> miarn_fiducial_data;

#include <iostream>

std::ostream& operator << ( std::ostream& os, miarn_fiducial_item& c );
std::ostream& operator << ( std::ostream& os, miarn_fiducial_data& c );

#endif
