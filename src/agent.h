// Reference : http:// argos-sim.info/examples.php

#ifndef AGENT_H
#define AGENT_H

#include <iostream>
#include <string>
#include <cmath>
//#include <algorithm>
#include "sensordata.h"

using namespace std;

const float PI = 3.1415927;
const float Epsilon=0.01;
const float TargetDistance=15; //Distance robot-robot in cm
const float Gain=1000;
const float Exponent=2;
const float BaseSpeed=1;
const float MaxSpeed=1;
const float SoftTurnThreshold = 0.175;
const float HardTurnThreshold = 1.222;


//const float ArenaWidth=231.14; //in X direction
//const float ArenaDepth=109.86; //in Y direction

//typedef pair<float,float> WheelSpeeds; //right speed, left speed
//typedef vector<SensorData> TVecData;

/* //from assignment 2, might be useful
struct LessThanByDistance {
    pair<float,float> temp;
    Distance(Coordinate point): temp(point) {}
    bool operator()(const Coordinate& obj1,const Coordinate& obj2){return ((pow(obj1.first-temp.first,2)+pow(obj1.second-temp.second,2))< (pow(obj2.first-temp.first,2)+pow(obj2.second-temp.second,2))) ;}
};

struct CompareXandYCoord {
    float x,y;
    CompareXandYCoord(float x,float y): x(x), y(y){}
    bool operator()(const Coordinate& obj){return abs(obj.first-x)<Epsilon && abs(obj.second-y)<Epsilon;}
};
*/

class Agent {

public:
    
    Agent();
    
    float GeneralizedLennardJones(float f_Distance);
    
    //SVector VectorToGoal(); //not needed
    
    SensorData FlockingVector();
    
    WheelSpeeds SpeedFromVector(SensorData  s_vector);
    
    WheelSpeeds ControlStep(Reading s_readings);
    
    SensorData AddVectors(SensorData inputA, SensorData inputB);

    private:
    SensorData m_tVectorToGoal;
    TVecData m_tReadings;
    //int m_nRobotId;
};

#endif
