#ifndef _XY_H_
#define _XY_H_

#include <cmath>

struct xy{
  float x;
  float y;
  xy(float a, float b):x(float(a)),y(float(b)){};
  xy(){};
  friend xy operator+ (const xy &op1,const xy & op2);
  friend xy operator- (const xy &op1,const xy & op2);
  inline xy rotate(float rad){
    xy temp;
    temp.x=x*cos(rad)-y*sin(rad);
    temp.y=x*sin(rad)+y*cos(rad);
    return temp;
  }
  ~xy(){};
};

inline xy operator+ (const xy &op1,const xy &op2){
  xy temp;
  temp.x=op1.x+op2.x;
  temp.y=op1.y+op2.y;
  return temp;
}

inline xy operator- (const xy &op1,const xy &op2){
  xy temp;
  temp.x=op1.x-op2.x;
  temp.y=op1.y-op2.y;
  return temp;
}

inline float distance(const xy &op1,const xy &op2){
  return sqrt((op1.x-op2.x)*(op1.x-op2.x)+(op1.y-op2.y)*(op1.y-op2.y));
}

#endif
