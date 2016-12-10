#ifndef SENSORDATA_H
#define SENSORDATA_H

#include <iostream>
#include <string>
#include <cmath>
#include <vector>
//#include "rbe510.h"

using namespace std;

struct SensorData {
	float distance;
	float theta;
    SensorData():distance(0), theta(0) {}
    SensorData(float distance,float theta): distance(distance), theta(theta) {}
};

struct Reading {
	int id;
	vector<SensorData> robotData;
	SensorData goalData;
	Reading(int id, vector<SensorData> robotData, SensorData goalData): id(id), robotData(robotData), goalData(goalData) {}
};

typedef vector<SensorData> TVecData;

typedef pair<float,float> WheelSpeeds; // first is left, second is right

#endif