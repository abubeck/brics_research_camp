#include "feature_person.hpp"

//#define DEBUG

FeatureLegTracker::FeatureLegTracker() : leg_update_radius(.5),
										 person_radius(1.0),
										 leg_clean_ticks(3),
										 person_clean_ticks(10)
{}

void FeatureLegTracker::LCReset() {
    #ifdef DEBUG
    printf("void FeatureLegTracker::LCReset()\n");
    #endif
    fdata_out.clear();
    /// clear candidates, 2legs
    list_candidates.clear();
    list_2legs.clear();

    /// put the leftovers of the previous instant incoming into the candidates list
    std::list<xy>::iterator it_clean;
    for (it_clean=list_incoming.begin();it_clean!=list_incoming.end();it_clean++) {
        list_candidates.push_back(*it_clean);
    }
    /// clear the incoming list
    list_incoming.clear();

    /// increase the last seen counters of the legs and the persons
    std::list<leg*>::iterator it_legs;
    for (it_legs=list_legs.begin(); it_legs!=list_legs.end(); it_legs++) {
        (**it_legs).seen_counter++;
    }
    std::list<Person*>::iterator it_persons;
    for (it_persons=list_persons.begin(); it_persons!=list_persons.end(); it_persons++) {
        (**it_persons).seen_counter++;
    }
    #ifdef DEBUG
    printf("void FeatureLegTracker::LCReset() end\n");
    #endif
}

/// parse incoming fiducials, namely "probable legs" and "probable two legs together"
/// and put them respectively in list incoming and list 2 legs
void FeatureLegTracker::CreateIncoming() {
    #ifdef DEBUG
    printf("void FeatureLegTracker::CreateIncoming()\n");
    #endif
    /// parse incoming fiducials, namely "probable legs" and "probable two legs together"
    /// and put them respectively in list incoming and list 2 legs
    for (uint i=0;i < fdata.size();i++) {
        switch (fdata[i].type) {
        case MIARN_FEATURE_TYPE_PLEG:
            list_incoming.push_back(xy(fdata[i].pos[0],fdata[i].pos[1]));
            break;
        case MIARN_FEATURE_TYPE_2LEGS:
            list_2legs.push_back(fdata[i]);
            break;
        }
    }
    #ifdef DEBUG
    printf("list_2legs.size()=%d\n",list_2legs.size());
    printf("list_incoming.size()=%d\n",list_incoming.size());
    printf("void FeatureLegTracker::CreateIncoming() end \n");
    #endif
}

/// visit legs and find a close neighbour
void FeatureLegTracker::LegMatchInTime() {
    #ifdef DEBUG
    printf("\n FeatureLegTracker::LegMatchInTime()");
    #endif
    /// possible two legs in ONE segment near any persons are added to legs list
    if (this->list_persons.size() > 0 && this->list_2legs.size() > 0 ) {
        std::list <Person*>::iterator it_persons,it_persons_min;
        std::list <miarn_fiducial_item>::iterator it_seg_leg,it_seg_leg_min;
        /// solve the two legs in one segment first
        /// split all the 2legs using the closest pairs to persons until the minimum distance is above a threshold
        do {
            float dist_tmp=1000000;
            // lets assume that the minimum distance to match is equal to the acceptance radius of a person
            float min_dist=this->person_radius;
            /// if the 2legs center is near a known person split it in two legs
            for (it_persons=list_persons.begin();it_persons!=list_persons.end();it_persons++) {
                for (it_seg_leg=this->list_2legs.begin();it_seg_leg!=this->list_2legs.end();it_seg_leg++) {
                    xy xy_tmp(it_seg_leg->pos[0],it_seg_leg->pos[1]);
                    if (( dist_tmp=distance(xy_tmp,(**it_persons).pos[1])) < min_dist ) {
                        it_seg_leg_min=it_seg_leg;
                        it_persons_min=it_persons;
                        min_dist=dist_tmp;
                    }
                }
            }
            /// when the shortest distance found is greater than the threshold of acceptance break cycle
            if (min_dist>=this->person_radius)
                break;
            /// move the two legs in one segment to the list of incoming legs
            else {
                xy inc1(it_seg_leg_min->extra[0],it_seg_leg_min->extra[1]);
                xy inc2(it_seg_leg_min->extra[2],it_seg_leg_min->extra[3]);
                list_incoming.push_back(inc1);
                list_incoming.push_back(inc2);
                list_2legs.erase(it_seg_leg_min);
            }
        } while (this->list_2legs.size() >0);
    }
    /// update legs with the incomings
    if ( list_legs.size() > 0) {
        std::list<xy>::iterator it_segm,it_segm_min;
        std::list<leg*>::iterator it_legs,it_legs_min;
        do {
            float minfound=this->leg_update_radius,temp;
            for (it_legs=list_legs.begin(); it_legs!=list_legs.end(); it_legs++) {
                /// if the leg has been updated already then continue
                if ((*it_legs)->seen_counter==0)
                    continue;
                for (it_segm = list_incoming.begin();it_segm != list_incoming.end();it_segm++) {
                    if ((temp=distance((**it_legs).pos[1],*it_segm)) < minfound) {
                        it_legs_min=it_legs;
                        it_segm_min=it_segm;
                        minfound=temp;
                    }
                }
            }
            if (minfound >= this->leg_update_radius)
                break;
            /// update a leg with the nearest incoming and remove that incoming
            (*it_legs_min)->update(*it_segm_min);
            list_incoming.erase(it_segm_min);
        } while (list_incoming.size() > 0);           // end if
    }
    #ifdef DEBUG
    printf("\n FeatureLegTracker::LegMatchInTime() end");
    #endif
}

/// close legs that weren't updated
void FeatureLegTracker::LegClean() {
    #ifdef DEBUG
    printf("\n FeatureLegTracker::LegClean(const float& age)");
    #endif

    std::list <leg*>::iterator it_legs;
    for (it_legs= list_legs.begin(); it_legs!= list_legs.end(); it_legs++) {
        /*
        // EXPERIMENTAL if the leg lies in a wide unoccupied area then clear it
        if (R(LASER_POINT_B(B_XY((*it_legs)->pos[1])))-R_XY((*it_legs)->pos[1]) < 200)
        {
          delete (*it_legs);
          list_legs.erase(it_legs--);
          continue;
        }
        */
        /// normally leg_clean_ticks == 1 ; remove leg if it wasn't updated
        if ((*it_legs)->seen_counter >= this->leg_clean_ticks) {
            delete (*it_legs);
            list_legs.erase(it_legs--);
            continue;
        }
    }
    #ifdef DEBUG
    printf("\n FeatureLegTracker::LegClean(const float& age) end");
    #endif
}

/// match the incomings with the candidates using closest pairs and remove candidate and incoming if a match happens
void FeatureLegTracker::LegCreate() {
    #ifdef DEBUG
    printf("\n FeatureLegTracker::LegCreate()");
    #endif
    std::list<xy>::iterator it_candidates,it_candidates_min;
    std::list<xy>::iterator it_incoming_leg,it_incoming_leg_min;
    if ( list_incoming.size() > 0 ) {
        do {
            float minfound=1000000,temp;
            for (it_candidates=list_candidates.begin(); it_candidates!=list_candidates.end(); it_candidates++) {
                for (it_incoming_leg=list_incoming.begin(); it_incoming_leg!=list_incoming.end();it_incoming_leg++) {
                    if ( (temp = distance(*it_candidates,*it_incoming_leg)) < minfound ) {
                        it_candidates_min=it_candidates;
                        it_incoming_leg_min=it_incoming_leg;
                        minfound=temp;
                    }
                }
            }
            if (minfound > this->leg_update_radius)
                break;
            else {
                list_legs.push_back(new leg(*it_incoming_leg_min,*it_candidates_min));
                list_candidates.erase(it_candidates_min);
                list_incoming.erase(it_incoming_leg_min);
            }
        } while ( list_incoming.size() > 0 );
    }
    #ifdef DEBUG
    printf("\n FeatureLegTracker::LegCreate()end");
    #endif
}

void FeatureLegTracker::PersonUpdate() {
    #ifdef DEBUG
    printf("\n void FeatureLegTracker::PersonUpdate() begin");
    #endif

    std::list <Person*>::iterator it_persons;
    for (it_persons=list_persons.begin();it_persons!=list_persons.end();it_persons++) {
        /// if person already updated skip
        if ( (*it_persons)->seen_counter==0)
            continue;
        /// increase the search radius r if person was not observed
        float r = this->person_radius
                  +float(this->person_radius)*((**it_persons).seen_counter-1)/this->person_clean_ticks;
        /// number of legs visible
        int nlegs=(int((**it_persons).lleg_ptr!=NULL) + int((**it_persons).rleg_ptr!=NULL));
        switch (nlegs) {
        case 2:
            #ifdef DEBUG
            printf("\n\n\n\n\n\n\n\n 2 LEGS");
            #endif
            ///if legs too spread
            if (distance((**it_persons).lleg_ptr->pos[1],((**it_persons).rleg_ptr->pos[1])) > this->person_radius) {
                ///if one leg moves and the other is still, center person on moving leg
                if (((**it_persons).lleg_ptr->dynamics == still) || ((**it_persons).rleg_ptr->dynamics == still)) {
                    leg* legptr;
                    ((**it_persons).lleg_ptr->dynamics==moving) ? legptr=(**it_persons).lleg_ptr : legptr=(**it_persons).rleg_ptr;
                    (**it_persons).pos[1].x= legptr->pos[1].x ;
                    (**it_persons).pos[1].y= legptr->pos[1].y ;

                    (**it_persons).pos[2].x = (**it_persons).pos[1].x;
                    (**it_persons).pos[2].y = (**it_persons).pos[1].y;
                    (**it_persons).pos[0]=(**it_persons).pos[1];
                    #ifdef DEBUG

                    printf("\n\n\n\n\n\n\n\n 2 LEGS far one moves");
                    #endif

                } else {
                    /// if both moving or still delete the person
                    delete *it_persons;
                    list_persons.erase(it_persons--);
                    #ifdef DEBUG
                    printf("\n\n\n\n\n\n\n\n 2 LEGS spread");
                    #endif
                    continue;
                }
            }
            break;
        case 1:
            #ifdef DEBUG
            printf("\n\n\n\n\n\n\n\n 1 LEGS");
            #endif

        case 0:
            #ifdef DEBUG
            printf("\n\n\n\n\n\n\n\n 0 LEGS");
            #endif
            /// if no legs were found search in incoming list for possible legs
            /// min1 is the first leg, min2 is the second leg if found
            if (list_incoming.size()>0) {                                        // do while
                float minfound1=1000000,minfound2=1000000,temp;
                std::list<xy>::iterator it_incoming_leg,it_incoming_leg_min1,it_incoming_leg_min2;
                for (it_incoming_leg=list_incoming.begin(); it_incoming_leg!=list_incoming.end(); it_incoming_leg++) {
                    if ((temp=distance(*it_incoming_leg,(**it_persons).pos[2])) < r) {
                        if (temp< minfound1) {
                            minfound1=temp;
                            it_incoming_leg_min1=it_incoming_leg;
                        } else if (temp < minfound2) {
                            minfound2=temp;
                            it_incoming_leg_min2=it_incoming_leg;
                        }
                    }
                }
                if (minfound1 < r) {
                    leg * a = new leg(*it_incoming_leg_min1, *it_incoming_leg_min1);
                    list_legs.push_back(a);
                    AttachLeg(*it_persons,a) ;
                    list_incoming.erase(it_incoming_leg_min1);
                    nlegs++;
                    #ifdef DEBUG
                    printf("\n        if (minfound1 < r)");
                    #endif

                }
                if (minfound2 < r && nlegs < 2) {
                    leg * a = new leg(*it_incoming_leg_min2, *it_incoming_leg_min2);
                    list_legs.push_back(a);
                    AttachLeg(*it_persons,a) ;
                    list_incoming.erase(it_incoming_leg_min2);
                    nlegs++;
                    #ifdef DEBUG
                    printf("\n        if (minfound2 < r && nlegs < 2)");
                    #endif

                }
            }
            break;
        }
        /// now that the legs are found lets update person position
        switch (nlegs) {
        case 2:
            (**it_persons).update();
            #ifdef DEBUG
            printf("\n(**it_persons).update();");
            #endif

            break;
        case 1:
            /// approximate person to the visible leg
            (**it_persons).seen_counter=0;
            leg* legptr;
            ((**it_persons).lleg_ptr!=NULL) ? legptr=(**it_persons).lleg_ptr : legptr=(**it_persons).rleg_ptr;
            (**it_persons).pos[0]=(**it_persons).pos[1];
            (**it_persons).pos[1].x= (2*legptr->pos[1].x + (**it_persons).pos[2].x)/3;
            (**it_persons).pos[1].y= (2*legptr->pos[1].y + (**it_persons).pos[2].y)/3;
            //(**it_persons).pos[1].x= legptr->pos[1].x;
            //(**it_persons).pos[1].y= legptr->pos[1].y;
            //(**it_persons).pos[2]=(**it_persons).pos[1];
            (**it_persons).pos[2].x = 2*(**it_persons).pos[1].x - (**it_persons).pos[0].x;
            (**it_persons).pos[2].y = 2*(**it_persons).pos[1].y - (**it_persons).pos[0].y;
            #ifdef DEBUG
            printf("\n case1");
            #endif

            break;
        case 0:
            (**it_persons).pos[0] = (**it_persons).pos[1];
            (**it_persons).pos[1] = (**it_persons).pos[2];
            (**it_persons).pos[2].x = 2*(**it_persons).pos[1].x -(**it_persons).pos[0].x;
            (**it_persons).pos[2].y = 2*(**it_persons).pos[1].y -(**it_persons).pos[0].y;
            #ifdef DEBUG

            printf("\n case0");
            #endif

            break;
        }
        /// if person not updated for a while remove it
        if ((**it_persons).seen_counter > this->person_clean_ticks) {
            delete *it_persons;
            list_persons.erase(it_persons--);
        }
    }
    #ifdef DEBUG
    printf("\n void FeatureLegTracker::PersonUpdate() end");
    #endif
}

void FeatureLegTracker::PersonCreate() {
    #ifdef DEBUG
    printf("\n FeatureLegTracker::PersonCreate()");
    #endif

    if ( list_legs.size() < 2 )
        return;
    float temp;
    std::list <leg*>::iterator it_legs1,it_legs2,it_legs_min1,it_legs_min2;
    do {
        float minfound=1000000;
        for (it_legs1=list_legs.begin();it_legs1!=list_legs.end();it_legs1++) {
            /// if leg is still or already belongs to person skip it
            if ((*it_legs1)->dynamics==still || (*it_legs1)->ptrPerson!=NULL)
                continue;
            for (++(it_legs2=it_legs1);it_legs2!=list_legs.end();it_legs2++) {
                /// if leg is still or already belongs to person skip it
                if ((*it_legs2)->dynamics==still || (*it_legs2)->ptrPerson!=NULL)
                    continue;
                if ( (temp=distance((**it_legs1).pos[1],(**it_legs2).pos[1])) < minfound ) {
                    it_legs_min1=it_legs1;
                    it_legs_min2=it_legs2;
                    minfound=temp;
                }
            }
        }
        if (minfound > this->person_radius)
            break;
        /// create a new person with the nearby legs found
        list_persons.push_back(new Person(*it_legs_min1, *it_legs_min2));
    } while ( list_legs.size() >= 2 );
    #ifdef DEBUG
    printf("\n FeatureLegTracker::PersonCreate() end");
    #endif
}

void FeatureLegTracker::FillFeature() {
    #ifdef DEBUG
    printf("\n void FeatureLegTracker::FillFeature()");
    #endif

    std::list <Person*>::iterator it_persons;
    for (it_persons=list_persons.begin();it_persons!=list_persons.end();it_persons++) {
        if ((**it_persons).lleg_ptr==NULL && (**it_persons).rleg_ptr==NULL) {
            AddPerson((**it_persons).pos[1].x,(**it_persons).pos[1].y,
                      (**it_persons).pos[0].x, (**it_persons).pos[0].y,
                      (**it_persons).pos[2].x, (**it_persons).pos[2].y,
                      0,0,
                      0,0
                     );
            /// left leg invisible
        } else if ((**it_persons).lleg_ptr==NULL) {
            AddPerson((**it_persons).pos[1].x,(**it_persons).pos[1].y,
                      (**it_persons).pos[0].x, (**it_persons).pos[0].y,
                      (**it_persons).pos[2].x, (**it_persons).pos[2].y,
                      0,0,
                      (**it_persons).rleg_ptr->pos[1].x, (**it_persons).rleg_ptr->pos[1].y
                     );
            /// right leg invisible
        } else if ((**it_persons).rleg_ptr==NULL) {
            AddPerson((**it_persons).pos[1].x,(**it_persons).pos[1].y,
                      (**it_persons).pos[0].x, (**it_persons).pos[0].y,
                      (**it_persons).pos[2].x, (**it_persons).pos[2].y,
                      (**it_persons).lleg_ptr->pos[1].x, (**it_persons).lleg_ptr->pos[1].y,
                      0,0
                     );
        } else {
            /// the person is visible
            AddPerson((**it_persons).pos[1].x,(**it_persons).pos[1].y,
                      (**it_persons).pos[0].x, (**it_persons).pos[0].y,
                      (**it_persons).pos[2].x, (**it_persons).pos[2].y,
                      (**it_persons).lleg_ptr->pos[1].x, (**it_persons).lleg_ptr->pos[1].y,
                      (**it_persons).rleg_ptr->pos[1].x, (**it_persons).rleg_ptr->pos[1].y
                     );
        }
    }
    /// add legs
    std::list <leg*>::iterator it_legs;
    for (it_legs=list_legs.begin();it_legs!=list_legs.end();it_legs++) {
        ///TODO:Add parameter to print all legs
        AddLeg((**it_legs).pos[1].x, (**it_legs).pos[1].y,
               (**it_legs).pos[0].x, (**it_legs).pos[0].y,
               (**it_legs).pos[2].x, (**it_legs).pos[2].y);
    }
}

void FeatureLegTracker::AddPerson(float pos_x, float pos_y,float past_pos_x, float past_pos_y,float future_pos_x, float future_pos_y, float lleg_x, float lleg_y,float rleg_x, float rleg_y) {
    miarn_fiducial_item feature;
    feature.pos[0]  = pos_x;
    feature.pos[1]  = pos_y;
    feature.begin_x = lleg_x;
    feature.begin_y = lleg_y;
    feature.end_x   = rleg_x;
    feature.end_y   = rleg_y;
    feature.extra[0]= future_pos_x;
    feature.extra[1]= future_pos_y;
    feature.extra[2]= past_pos_x;
    feature.extra[3]= past_pos_y;
    feature.type    = MIARN_FEATURE_TYPE_PERSON;
    fdata_out.push_back(feature);
}

void FeatureLegTracker::AddLeg(float pos_x, float pos_y,float past_pos_x, float past_pos_y,float future_pos_x, float future_pos_y) {
    miarn_fiducial_item feature;
    feature.pos[0]  = pos_x;
    feature.pos[1]  = pos_y;
    feature.extra[0]= future_pos_x;
    feature.extra[1]= future_pos_y;
    feature.extra[2]= past_pos_x;
    feature.extra[3]= past_pos_y;
    feature.type    = MIARN_FEATURE_TYPE_LEG;
    fdata_out.push_back(feature);
}

leg::leg(const xy& new_pos,const xy& xyold) {
    this->ptrPerson=NULL;
    this->born=xyold;
    this->seen_counter=0;
    this->pos[0]=xyold;
    this->pos[1]=new_pos;
    if (distance(this->pos[1],this->born) > 0.1) this->dynamics=moving ;
    else this->dynamics=still;

    this->pos[2].x = 2*this->pos[1].x - this->pos[0].x;
    this->pos[2].y = 2*this->pos[1].y - this->pos[0].y;
}

void leg::update(const xy& new_pos) {
    this->pos[0]=this->pos[1];
    this->pos[1]=new_pos;
    this->seen_counter=0;
    if (dynamics==still) {
        if (distance(this->pos[1],this->born) > 0.1) this->dynamics=moving;
    }
    this->pos[2].x = 2*this->pos[1].x - this->pos[0].x;
    this->pos[2].y = 2*this->pos[1].y - this->pos[0].y;
}

leg::~leg() {
    if (ptrPerson!=NULL) {
        if (this->ptrPerson->lleg_ptr==this) {
            this->ptrPerson->lleg_ptr=NULL;
        }

        if (this->ptrPerson->rleg_ptr==this) {
            this->ptrPerson->rleg_ptr=NULL;
        }
    }
}

Person::Person(leg * lleg, leg * rleg) {
    lleg_ptr=NULL;
    rleg_ptr=NULL;
    AttachLeg(this,lleg);
    AttachLeg(this,rleg);
    seen_counter=0;
    pos[1].x=(lleg_ptr->pos[1].x+rleg_ptr->pos[1].x)/2;
    pos[1].y=(lleg_ptr->pos[1].y+rleg_ptr->pos[1].y)/2;
    pos[0]=pos[2]=pos[1];
}

Person::~Person() {
    if (this->lleg_ptr!=NULL)
        lleg_ptr->ptrPerson=NULL;
    if (this->rleg_ptr!=NULL)
        rleg_ptr->ptrPerson=NULL;
}

void Person::update() {
    pos[0]=pos[1];
    this->seen_counter=0;
    pos[1].x=(lleg_ptr->pos[1].x+rleg_ptr->pos[1].x)/2;
    pos[1].y=(lleg_ptr->pos[1].y+rleg_ptr->pos[1].y)/2;
    pos[2].x = 2*pos[1].x-pos[0].x;
    pos[2].y = 2*pos[1].y-pos[0].y;
}

bool AttachLeg(Person* a, leg* b) {
    if (a->lleg_ptr==NULL) {
        a->lleg_ptr=b;
        b->ptrPerson=a;
        return 1;
    } else if (a->rleg_ptr==NULL) {
        a->rleg_ptr=b;
        b->ptrPerson=a;
        return 1;
    } else {
        return 0;
    }
}

bool DetachLeg(Person* a, leg* b) {
    if (a->lleg_ptr==b) {
        a->lleg_ptr=NULL;
        b->ptrPerson=NULL;
        return 1;
    } else if (a->rleg_ptr==NULL) {
        a->rleg_ptr=b;
        b->ptrPerson=NULL;
        return 1;
    } else return 0;
}
