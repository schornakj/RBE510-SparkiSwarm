// Reference : http:// argos-sim.info/examples.php

#ifndef AGENT_H
#define AGENT_H

#include <iostream>
#include <string>
#include <cmath>
//#include <algorithm>

using namespace std;

const float PI = 3.1415927;
const float ArenaWidth=231.14; //in X direction
const float ArenaDepth=109.86; //in Y direction
const float Epsilon=0.01;

struct SensorData {
    float distance;
    float theta;
}

//typedef pair<float, float> Direction; // (norm, angle)
typedef pair<float,float> WheelSpeeds;
typedef vector<Coordinate> TVecCoord;
typedef vector<SensorData> TVecData;

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
    
    float GeneralizedLennardJones();
    
    //float VectorToGoal(); //not needed, simply get the angle reading to the
    
    float FlockingVector();
    
    WheelSpeeds SpeedFromVector();
    
    private:
    SensorData m_tVectorToGoal;
    TVecData m_tVectorsToRobots;
    //int m_nRobotId;
};

#endif
