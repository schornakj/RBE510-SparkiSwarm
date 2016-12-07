#ifndef SENSOR_H
#define SENSOR_H

#include <iostream>
#include <string>
#include <cmath>
#include "rbe510.h"
#include "sensordata.h"

using namespace std;

float sensorThreshold = 30;

Robot getRobot(int id, FieldData data){
    	Robot robot(id);
    	bool found = false;
    	for(unsigned i = 0; i < data.robots.size(); i++){
        	if(data.robots[i].id() == robot.id()) {
            	robot = data.robots[i];
            	return robot;
        	}
    	}
    	return Robot(-1);
	}

vector<SensorData> SimulateSensor(int inputID, FieldData inputData) {
	// Use field data derived from the server's computer vision algorithm to measure the distance to neighboring robots.
	vector<SensorData> output;
	
	Robot thisRobot = getRobot(inputID, inputData);
	
	for(unsigned i = 0; i < inputData.robots.size(); i++){	
		if(data.robots[i].id() != inputID) {
			float distance = sqrt(pow(data.robots[i].x() - thisRobot.x(),2) + pow(data.robots[i].y() - thisRobot.y(),2);
		
			if (distance <= sensorThreshold){
				float angle = atan2(data.robots[i].y() - thisRobot.y(), data.robots[i].x() - thisRobot.x());
				output.push_back(SensorData(distance, angle));
			}
		}
	}
	return output;
}

vector<Reading> GetRobotSensorReading(int inputID, vector<Reading> readings) {
	for (unsigned i = 0; i < readings.size(); i++) {
		if (readings[i].id == inputID) {
			return readings[i];
	}
}

#endif