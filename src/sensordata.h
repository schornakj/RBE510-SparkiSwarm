#ifndef SENSORDATA_H
#define SENSORDATA_H

#include <iostream>
#include <string>
#include <cmath>

struct SensorData {
	float distance;
	float theta;
};

struct Reading {
	int id;
	vector<SensorData> robotData;
	SensorData goalData;
};

typedef vector<SensorData> TVecData;

typedef pair<float,float> wheelSpeeds; // first is left, second is right

SensorData AddVectors(SensorData inputA, SensorData inputB) {
	float xA, xB, xC, yA, yB, yC;
	
	xA = inputA.distance*cos(inputA.theta);
	xB = inputB.distance*cos(inputB.theta);
	
	yA = inputA.distance*sin(inputA.theta);
	yB = inputB.distance*sin(inputB.theta);
	
	xC = xA + xB;
	yC = yA + yB;
	
	return SensorData(sqrt(pow(xC,2)+pow(yC,2)), atan2(yC,xC));
}

#endif