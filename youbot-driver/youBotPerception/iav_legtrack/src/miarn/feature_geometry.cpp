#include "feature_geometry.hpp"
#include <stdio.h>

LaserFeatureX::LaserFeatureX() : max_laser_range(8),
								 arc_min_aperture(1.57), arc_max_aperture(2.365),
								 arc_std_max(0.25),
								// segmentation_threshold(.200),
								segmentation_threshold(0.001),
								 line_min_distance(.120), line_error_threshold(.020),
								 max_leg_diameter(.200), min_leg_diameter(.040),
								 do_circles(true), do_lines(true), do_legs(true), iav_do_lines(true),
								 safe_circle_corners(false)
{}

float LaserFeatureX::Average(float a[],int size_a) {
    float sum = 0;
    for (int i = 0;i < size_a;i++) {
        sum = sum + a[i];
    }
    return sum / size_a;
}

float LaserFeatureX::StandardDeviation(float a[],int size_a,float meanvalue) {
    float totalvar = 0;                            //float to total array values to find the variant
    int array_size = size_a;
    int count = size_a;
    while (count > 0)                               //loop to add totalvar to be used in variant formula
    {
        count--;
        totalvar = totalvar + ((a[count] - meanvalue) * (a[count] - meanvalue));
    }
    float variant = totalvar / --array_size ;      //formula for variant
    return sqrt(variant);                           //formula for standard
}

/// find arc in segment
bool LaserFeatureX::FitArc(int begin,int end) {
    #ifdef ARC_DEBUG
    printf("\n begin: %d -> %d", begin,end);
    #endif
    /// if doesn't obey the circle prerequisites return false
    if (!this->CirclePreRequisites(begin,end))
        return false;
    /// the border points contain more reflection errors, avoid them if more than 20 points in segment
    if (end-begin >20) {
        end--;
        begin++;
    };

    /// create some variables
    int segment_size= end-begin+1;
    xy& xy_right=this->point_xy[begin];
    xy& xy_left=this->point_xy[end];
    int slopes_size=segment_size-3;
	float *slopes = new float[slopes_size];
    float ma,mb;

    /// make vector of slopes
    for (int i=0; i<slopes_size;i++) {
        xy xy_temp=this->point_xy[begin+i+1];
        ma=atan2(xy_left.y-xy_temp.y,xy_left.x-xy_temp.x);
        mb=atan2(xy_right.y-xy_temp.y,xy_right.x-xy_temp.x);
        slopes[i]=ma-mb;
        #ifdef ARC_DEBUG
        printf("\nxy_temp(%f,%f) ma -mb = %f - %f = slopes[%d]=%f",xy_temp.x,xy_temp.y,ma,mb,i,slopes[i]);
        #endif
    }

    /// avoid divide by zero
    if (ma==0)
        ma=1e-8;                             //0.0000001;
    if (mb==0)
        mb=1e-8;                             //0.0000001;

    /// calculate average and standard deviation
    float average, std_dev;
    average=Average(slopes,slopes_size);
    std_dev=StandardDeviation(slopes,slopes_size,average);
    #ifdef ARC_DEBUG
    printf(" average = %f, std = %f",average,std_dev);
    #endif

    /// hack to prevent straight corners to be detected as arcs
    if (safe_circle_corners) {
        float min=10000;
        int min_idx;
        // if is a corner exclude it

        if (slopes_size > 4) {
            for (int temp=0;temp<slopes_size;temp++) {
                if (slopes[temp] < min) {
                    min=slopes[temp];
                    min_idx=temp;
                } else
                    continue;
            }
            if (slopes_size-min_idx>1 && min_idx > 1) {
                if (slopes[min_idx-2] > slopes[min_idx-1] &&
                        slopes[min_idx-1] > slopes[min_idx]   &&
                        slopes[min_idx+1] > slopes[min_idx]   &&
                        slopes[min_idx+2] > slopes[min_idx+1]) {
                    #ifdef ARC_DEBUG
                    printf(" corner in %d:%d with %f > %f > %f < %f < %f",begin,end,slopes[min_idx-2] ,slopes[min_idx-1] ,slopes[min_idx], slopes[min_idx+1] ,slopes[min_idx+2] );
                    #endif

					delete [] slopes;
                    return false;// is a corner
                };
            };
        };
    };
    /// was it a line ?
    if (std::abs(average-M_PI) < .1 && std_dev < 0.2) {
        /// are we searching for lines ?
        if (this->iav_do_lines) {
            float m,b,r;
            this->LinearRegression(m,b,r,begin,end);
            if (m==0 && b==0 && r==0) {
                return false;
            };
            if (r < this->line_error_threshold) {
                this->AddLine(m,b,r, point_xy[begin].x, point_xy[end].x, point_xy[begin].y, point_xy[end].y, begin, end);
            };
            #ifdef ARC_DEBUG
            printf("\n  circle detection detected line");
            #endif

            //return MIARN_FEATURE_TYPE_LINE;
			return false;
        } else {
            #ifdef ARC_DEBUG
            printf(" circle detection detected line ");
            #endif

            return false;
        };
    };

    /// is the std low(possible arc)
    if (std_dev < this->arc_std_max) {
        ///Q:find center and radius
        ///     |x
        ///     |O
        ///    /|\
        ///   / | \
        ///  / L|  \
        /// /q__|___\______y
        /// B   M   A
        /// find the center of the circle
        /// given: IAV, B, A
        ///
        ///A:we know from Euclid that BOA=360-2*IAV
        ///  we also know that BOA+2*q=180
        ///  solving : angle q = IAV-90�
        ///  tranlate the triangle to the origin, rotate so that BA are in XX
        ///  calculate L=dist(BM)*tg(q)
        ///  rotate and translate back from origin
        xy tmp=xy_right-xy_left;
        float angle_to_rotate = atan2(tmp.y,tmp.x);
        tmp=tmp.rotate(-angle_to_rotate);
        float middle=tmp.x/2.0;
        float q=average-1.5708;
        float height=middle*tan(q);
        xy center(middle,height);
        float radius=float(sqrt(center.y*center.y+center.x*center.x));
        center=center.rotate(angle_to_rotate);
        center= center+xy_left;
        float xa=float(center.x);
        float ya=float(center.y);
        #ifdef ARC_DEBUG
        /// display not so well defined circles
        if (std_dev>.10 && std_dev<.50) {
            printf("\n Dubious ARC xa=%f, ya=%f,radius=%f",xa,ya,radius);
            printf("\n Average = %f, std = %f",average,std_dev);
            printf("\n slopes=[%f",slopes[1]);
            for (int temp=0;temp<slopes_size;temp++) {
                printf("\n %f",slopes[temp]);
            }
            ;                                          //for
            printf("\n]");
        } else
            printf ("\n xa=%d, ya=%d,radius=%d",xa,ya,radius);
        #endif

		delete [] slopes;

        /// if can be circle add to feature list
        if ( this->arc_max_aperture > average &&      //2.375 > average &&//2.35 > average &&
                average > this->arc_min_aperture) {
            AddArc(xa,ya,radius, point_xy[begin].x, point_xy[end].x, point_xy[begin].y, point_xy[end].y,begin, end,average,std_dev);
            return true;
        } else
		return false;
    }
    return false;
}

void LaserFeatureX::Segmentation() {
    this->segments.clear();
    int mask=0;                                     // mask == 0 means no segment active
    segment segment_tmp(0,this->ranges.size()-1);

    for (uint i = 0; i < this->ranges.size(); i++) {
        if (this->ranges[i] >= this->max_laser_range) {
            if (mask==1) {
                if ( segments.size() > 0 ) { 
                    this->segments.at(segments.size()-1).end = i-1;
                }
            };
            mask=0;
            continue;
        };
        if (mask==0) { // create a new segment
            segment_tmp.begin = i;
            mask=1;
        };
        if (std::abs(this->ranges[i]-this->ranges[i+1]) > this->segmentation_threshold) { // check if should close segment
            segment_tmp.end = i;
            this->segments.push_back(segment_tmp);
            segment_tmp.end = this->ranges.size()-1;
            mask=0;
        };
    }


    for (uint i = 0; i < this->segments.size(); i++) {
        this->AddSegment(point_xy[segments[i].begin].x, point_xy[segments[i].end].x,
                         point_xy[segments[i].begin].y, point_xy[segments[i].end].y,
                         segments[i].begin, segments[i].end);

	segbeginx[i] = point_xy[segments[i].begin].x;	//connetction to arc_detection.cpp
	segbeginy[i] = point_xy[segments[i].begin].y;	//connetction to arc_detection.cpp
	segendx[i] = point_xy[segments[i].end].x;	//connetction to arc_detection.cpp
	segendy[i] = point_xy[segments[i].end].y;	//connetction to arc_detection.cpp

	}
}

/// quick check if this could be a circle
bool LaserFeatureX::CirclePreRequisites(const int begin,const int end) {
    if (end-begin < 4  )
        return false;              //|| !(this->do_circles)
    #ifdef ARC_DEBUG
    printf("\nAnalysing segment %d : %d for circles",begin,end);
    #endif

    int middle_point = (end+begin)/2;
    xy rotated_point=this->point_xy[middle_point];
    xy xy_right=this->point_xy[begin];
    xy xy_left=this->point_xy[end];
    rotated_point=rotated_point-xy_left;
    float angle_to_rotate = atan2((xy_left.x-xy_right.x)/(xy_left.y-xy_right.y),1);
    rotated_point=rotated_point.rotate(angle_to_rotate);
    if (-rotated_point.x > .1 * distance(xy_left,xy_right) &&
            -rotated_point.x <      distance(xy_left,xy_right)   ) {
        return true;
    } else
        return false;
}

bool LaserFeatureX::LinePreRequisites(const int& begin,const int& end) {
    if (end-begin > 4 && distance(point_xy[begin],point_xy[end]) >this->line_min_distance) {
        return true;
    }
    return false;
}

bool LaserFeatureX::FindLeg(const int begin,const int end, bool check2legs) {
    if (end-begin==0)
        return 0;
    float diameter = distance(point_xy[begin], point_xy[end]);
    /// is this segment diameter too wide even for 2 legs ?
    if ( diameter > 2*this->max_leg_diameter)
        return 0;
    /// could this segment represent two legs together ?
    int tmp_legmid=int(begin+end)/2;

    if ( diameter > this->max_leg_diameter && check2legs) {
        uint tmp_leg1,tmp_leg2;
        tmp_leg1=uint(begin+0.25*(end-begin));
        tmp_leg2=uint(begin+0.75*(end-begin));
        AddProbableTwoLegs(
            point_xy[tmp_legmid].x, point_xy[tmp_legmid].y,
            point_xy[tmp_leg1].x  , point_xy[tmp_leg1].y  ,
            point_xy[tmp_leg2].x  , point_xy[tmp_leg2].y  ,
            point_xy[begin].x     , point_xy[end].x       ,
            point_xy[begin].y, point_xy[end].y,
            begin, end);
        #ifdef DEBUG
        printf("\nadded 2legs %d %d", begin, end);
        #endif
        return 1;
    }
    /// this can be one leg
    else if (!( (this->ranges[begin-1] < this->ranges[begin]
				 && this->ranges[end+1] < this->ranges[end])
                || diameter < this->min_leg_diameter)
			 && !check2legs) {
        AddProbableLeg(point_xy[tmp_legmid].x, point_xy[tmp_legmid].y,point_xy[begin].x, point_xy[end].x,point_xy[begin].y, point_xy[end].y,begin, end);
        return 1;
    } else
        return 0;
}

/// define a line
void LaserFeatureX::lineDef(float &A,float &B,float &C, const int begin,const int end) {
    //Ax+By+C=0
    float m1,m2;
    m1=this->point_xy[begin].y-this->point_xy[end].y;
    m2=this->point_xy[begin].x-this->point_xy[end].x;

    A = m1;
    B = -m2;
    C = m2*this->point_xy[end].y - m1*this->point_xy[end].x;
}

void LaserFeatureX::RecursiveLineFitting(int begin, int end) {
    if (!this->LinePreRequisites(begin,end))
        return;
    float m,b,r;
    LinearRegression(m,b,r,begin,end);
    if (m==0 && b==0 && r==0) {
        return;
    };
    if (r < this->line_error_threshold) {
        AddLine(m, b,r, point_xy[begin].x, point_xy[end].x, point_xy[begin].y, point_xy[end].y, begin, end);
    } else {
        // The points don't fit
        float A,B,C;
        lineDef(A,B,C,begin,end);
        // search for break lines
        int nBreakBeam = 0;
        float dist,distMax=0;
        for (int point=begin; point < end; point++) {
            dist = fabs( (point_xy[point].x*A + point_xy[point].y*B + C)/sqrt(A*A + B*B) );
            if (dist > distMax) {
                distMax = dist;
                nBreakBeam = point;
            };
        }
        if (distMax > this->line_error_threshold) {
            this->RecursiveLineFitting(begin, nBreakBeam);
            this->RecursiveLineFitting(nBreakBeam,end);
        };
    }                                               // end if (r > this->line_error_threshold)
}


void LaserFeatureX::LinearRegression(float & m,float &b, float &r, int begin, int end) {
    int nPoints = (int) std::abs(float(end-begin))+1;
    if (nPoints > 0) {
        float SumX  = 0;
        float SumY  = 0;

        for (int n=begin; n<=end; n++) {
            SumX  = SumX  + this->point_xy[n].x;
            SumY  = SumY  + this->point_xy[n].y;
        }

        float Xmed= SumX/nPoints;
        float Ymed= SumY/nPoints;

		xy *xy_average = new xy[nPoints];
        xy temp_xy;
        for (int n=begin; n<=end; n++) {
            temp_xy.x=this->point_xy[n].x - Xmed;
            temp_xy.y=this->point_xy[n].y - Ymed;
            xy_average[n-begin]=temp_xy;
        }

        float A = 0;
        float SumXY = 0;
        for (int n=0; n<nPoints; n++) {
            A = A + (xy_average[n].x*xy_average[n].x - xy_average[n].y*xy_average[n].y);
            SumXY = SumXY + (xy_average[n].x*xy_average[n].y);
        }

        if (SumXY==0)
            SumXY=1e-8;

        A = A / SumXY;
        // m^2 + A*m - 1 = 0
        float m1 = (-A + sqrt(A*A + 4))/2;
        float m2 = (-A - sqrt(A*A + 4))/2;

        float b1 = Ymed - m1*Xmed;

        A=m1;
        float B=-1;
        float C=b1;

        //calculate the maximum error
        r=0;
        float aux = sqrt(A*A + B*B);
        float dist;
        for (int n=begin; n<end; n++) {
            dist = fabs( (this->point_xy[n].x *A + this->point_xy[n].y*B + C)/aux );
            if (dist > r)
                r = dist;
        }

        float r1 = r;

        float b2 = Ymed - m2*Xmed;
        A=m2;
        B=-1;
        C=b2;

        //calculate the maximum error
        r=0;
        aux = sqrt(A*A + B*B);
        for (int n=begin; n<end; n++) {
            dist = fabs((this->point_xy[n].x *A + this->point_xy[n].y*B + C)/aux );
            if (dist > r)
                r = dist;
        }

        float r2=r;

        if (r1 > r2) {
            m=m2;
            b=b2;
            r=r2;
            //             printf("if m=%f,b=%f,r=%f\n",m,b,r);
        } else {
            m=m1;
            b=b1;
            r=r1;
            // 	    printf("else m=%f,b=%f,r=%f\n",m,b,r);
        };

		delete [] xy_average;

    } else {
        m=0;
        b=0;
        r=0;
    }
	
}

void LaserFeatureX::AddProbableTwoLegs(float pos_x, float pos_y,float leg1_x,float leg1_y,float leg2_x,float leg2_y,float begin_x, float end_x, float  begin_y, float end_y,int begin_index, int end_index) {
//     miarn_fiducial_item * feature;
    miarn_fiducial_item feature;
//     feature = this->fdata.fiducials + this->fdata.count++;
    feature.begin_index = begin_index;
    feature.end_index = end_index;
    feature.begin_x=begin_x;
    feature.begin_y=begin_y;
    feature.end_x=end_x;
    feature.end_y=end_y;
    feature.pos[0]=pos_x;
    feature.pos[1]=pos_y;
    feature.extra[0]=leg1_x;
    feature.extra[1]=leg1_y;
    feature.extra[2]=leg2_x;
    feature.extra[3]=leg2_y;
    feature.type = MIARN_FEATURE_TYPE_2LEGS;
    fdata.push_back(feature);
}

void LaserFeatureX::AddProbableLeg(float pos_x, float pos_y,float begin_x, float end_x, float  begin_y, float end_y,int begin_index, int end_index) {
//     miarn_fiducial_item * feature;
    miarn_fiducial_item feature;
//     feature = this->fdata.fiducials + this->fdata.count++;
    feature.begin_index = begin_index;
    feature.end_index = end_index;
    feature.begin_x=begin_x;
    feature.begin_y=begin_y;
    feature.end_x=end_x;
    feature.end_y=end_y;
    feature.pos[0]=pos_x;
    feature.pos[1]=pos_y;
    feature.type = MIARN_FEATURE_TYPE_PLEG;
    fdata.push_back(feature);
}

void LaserFeatureX::AddSegment(float begin_x, float end_x, float  begin_y, float end_y,int begin_index, int end_index) {
    miarn_fiducial_item feature;
    feature.begin_index = begin_index;
    feature.end_index = end_index;
    feature.begin_x=begin_x;
    feature.begin_y=begin_y;
    feature.end_x=end_x;
    feature.end_y=end_y;
    feature.type = MIARN_FEATURE_TYPE_SEGMENT;
    fdata.push_back(feature);
}

void LaserFeatureX::AddLine(float slope, float offset,float maxerror, float begin_x, float end_x, float  begin_y, float end_y,int begin_index, int end_index) {
//     miarn_fiducial_item * feature;
    miarn_fiducial_item feature;
//     feature = this->fdata.fiducials + this->fdata.count++;
    feature.begin_index = begin_index;
    feature.end_index = end_index;
    feature.begin_x=begin_x;
    feature.begin_y=begin_y;
    feature.end_x=end_x;
    feature.end_y=end_y;
    /// distance from the origin to the normal (rho)
    float rho=fabs(offset/sqrt(slope*slope+1));
    /// angle from the origin to the normal (theta)
    float theta=atan2(-1/slope,1);
    if ((slope > 0 && offset > 0) || (slope < 0 && offset < 0)) {
        theta=-(M_PI-theta);
    };
    feature.pos[0]  =rho*cos(theta);
    feature.pos[1]  =rho*sin(theta);
    feature.extra[0]=slope*1e3;
    feature.extra[1]=offset;
    feature.extra[2]=maxerror;
    feature.type = MIARN_FEATURE_TYPE_LINE;
    fdata.push_back(feature);
}

void LaserFeatureX::AddArc(float pos_x, float pos_y, float radius, float begin_x, float end_x, float  begin_y, float end_y,int begin_index, int end_index,float average, float std_dev) {
    miarn_fiducial_item feature;
//     miarn_fiducial_item * feature;
//     feature = this->fdata.fiducials + this->fdata.count++;
    feature.begin_index = begin_index;
    feature.end_index = end_index;
    feature.begin_x=begin_x;
    feature.begin_y=begin_y;
    feature.end_x=end_x;
    feature.end_y=end_y;
    feature.pos[0]=pos_x;
    feature.pos[1]=pos_y;
    feature.extra[0]=radius;
    feature.extra[1]=average*1e3;
    feature.extra[2]=std_dev*1e3;
    feature.type = MIARN_FEATURE_TYPE_ARC;
    fdata.push_back(feature);
}

/*
///TODO:extend line, circle
void ExtendFeatures(){
/// if requested extend features

/// for each circle see if adjacent points fit in circle formula, if they fit extend feature
for (int i = 0; i < this->fdata.count; i++) {
if (this->fdata.fiducials[i].type==MIARN_FEATURE_TYPE_ARC){
/// go left
if left <= 0 continue
/// (x-x1)^2+(y-y1)^2=r^2
x=point_xy[i]-this->fdata.fiducials[i].value[2];
xx=x*x;
rr=
point_xy[i]-this->fdata.fiducials[i].value[2] *
point_xy[i]-this->fdata.fiducials[i].value[2]

point_xy[i]-this->fdata.fiducials[i].value[3]

if error <= abs(formula do circulo)
fdata.fiducials[i].[1--]
point_xy[i]
this->fdata.fiducials[i].type
this->fdata.fiducials[i].value[j]
if right >=ldata.range_count continue
fdata.fiducials[i].[1++]
}


}
/// for each line see if adjacent points fit in line formula, if they fit extend feature

};
*/
