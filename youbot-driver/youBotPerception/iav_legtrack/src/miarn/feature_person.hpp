#ifndef _FEATURE_PERSON_HPP_
#define _FEATURE_PERSON_HPP_


/**
@par Configuration file options
 
- inc_fid_index (integer)
- default: 0
- incoming Feature index

- leg_clean_ticks (integer)
- default: 1
- ticks needed to remove legs

- person_clean_ticks (integer)
- default: 10
- ticks needed to remove persons

- leg_update_radius (float)
- default: 0.6
- radius to search for the same leg in mm

- person_radius (float)
- default: 0.6
- radius to search for the same person in mm

Joao Xavier smogzer_at_gmail.com
*/

#include <list>
#include <cmath>

#include "xy.hpp"
#include "fiducial.hpp"

typedef unsigned int uint;

enum dynamic_t{still, moving};
struct Person;
struct leg;
bool AttachLeg(Person* a, leg* b);
bool DetachLeg(Person* a, leg* b);

struct Person {
    Person(leg * lleg, leg * rleg);
    Person() {}
    ~Person();

    int seen_counter;
    xy pos[3]; /* If both legs are attached, then
                  The person coordinate is the midpoint of the two attached legs (i.e., lleg_ptr and rleg_ptr).
                  pos[0] - previous person center coordinate
                  pos[1] - current person center (i.e., midpoint of currently attached legs).
                  pos[2] - predicted next position (using same technique as for legs, i.e.,
                                                    pos[2].x = pos[1].x + (pos[1].x-pos[0].x)
                                                    pos[2].y = pos[1].y + (pos[1].y-pos[0].y) )

                  If only one leg is attached, then
                  pos[0] - previous person center coordinate
                  pos[1] - weighted sum of current leg coordinate (call it L)
                           and previously predicated coordinate; i.e.,
                           pos[1] := (2/3)*L + (1/3)*pos[2] (where pos[2] is from
                                                             the last person update).
                  pos[2] - predicted next position using the same technique as always, but note
                           that the prediction is based on pos[0] and pos[1] as defined above
                           (that is, it is based on other estimates since there is only one leg here).

                  If no legs are attached, then
                  pos[0] - previous person center coordinate
                  pos[1] - previously predicted position (i.e., pos[2] from last person update).
                  pos[2] - predicted next position using the same technique as always, but note
                           that the prediction is based on pos[0] and pos[1] as defined above
                           (that is, the prediction is an estimate of an estimate and thus we
                            have increased uncertainty).
			   */
    /// pointers to the person leg
    leg* lleg_ptr, *rleg_ptr;

    void update();
};

struct leg {
    leg() {}
    ~leg();

    //int lbegin, lend;
    //int leg_diameter;
    /// is it moving ?
    dynamic_t dynamics;
    /// last seen in ?
    int seen_counter;
    /// born position
    xy born;
    /// current position
    xy pos[3]; /* pos[0] - previous leg position
                  pos[1] - current leg position
                  pos[2] - predicted next point; by linear extrapolation, i.e.,
                                                 pos[2].x = pos[1].x + (pos[1].x-pos[0].x)
                                                 pos[2].y = pos[1].y + (pos[1].y-pos[0].y)
				*/
    /// the person that owns the leg
    Person * ptrPerson;
    void update(const xy& new_pos);         //, const int& lbeg,const int& lend);//, const int& diam
    leg(const xy& new_pos,const xy& xyold); //,const int& lbeg,const int& lend);//, const int& diam);
};

struct FeatureLegTracker{

	FeatureLegTracker();

    bool UpdateFeatureData();
    void FillFeature();

    void LegMatchInTime();
    void LegClean();
    void LegCreate();
    void PersonUpdate();
    void PersonCreate();
    void LCReset();
    void CreateIncoming();

    float leg_update_radius;
    float person_radius;
    int leg_clean_ticks;
    int person_clean_ticks;

    /// Lists of structures
    std::list<xy> list_candidates;
    std::list<xy> list_incoming;
    std::list<miarn_fiducial_item> list_2legs;
    std::list<leg*> list_legs;
    std::list<Person*> list_persons;

    /// Add the structures to fiducial list
    void AddPerson(float pos_x, float pos_y,
				   float past_pos_x, float past_pos_y,
				   float future_pos_x, float future_pos_y,
				   float lleg_x, float lleg_y, float rleg_x, float rleg_y );
    void AddLeg(float pos_x, float pos_y,
				float past_pos_x, float past_pos_y,
				float future_pos_x, float future_pos_y );

    miarn_fiducial_data fdata;
	miarn_fiducial_data fdata_out;
};


#endif
