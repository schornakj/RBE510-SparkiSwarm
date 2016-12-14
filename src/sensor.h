#ifndef SENSOR_H
#define SENSOR_H

#include <iostream>
#include <string>
#include <cmath>
#include "rbe510.h"
#include "sensordata.h"

using namespace std;


// Get a specific Robot using its ID and the field data
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


Entity getEntity(int id, FieldData data){
	Entity entity(id);
	bool found = false;
	for(unsigned i = 0; i < data.entities.size(); i++){
    	if(data.entities[i].id() == entity.id()) {
        	entity = data.entities[i];
        	return entity;
    	}
	}
	return Entity(-1);	
}




// For a specific robot, measure the distances and angles to all its neighbors. Return a vector of readings for neighbors closer than the specified threshold.	
vector<SensorData> SimulateSensor(int inputID, FieldData data, float sensorThresholdCm) {
	// Use field data derived from the server's computer vision algorithm to measure the distance to neighboring robots.
	//cout << "Simulating sensor" << endl;
	vector<SensorData> output;
	
	float pixelsPerCm = (getEntity(202, data).x() - getEntity(200,data).x())/231.14;
	//cout << "Px/Cm: " << pixelsPerCmNew << endl;
	//float pixelsPerCm = 1;

	Robot thisRobot = getRobot(inputID, data);

	cout << "Metric Position: " <<thisRobot.x()/pixelsPerCm << " " << thisRobot.y()/pixelsPerCm << endl;
	
	for(unsigned i = 0; i < data.robots.size(); i++){	
		if(data.robots[i].id() != inputID) {
			float distance = sqrt(pow(data.robots[i].x() - thisRobot.x(),2) + pow(data.robots[i].y() - thisRobot.y(),2))/pixelsPerCm;
		
			if (distance <= sensorThresholdCm){
				float angle = atan2(data.robots[i].y() - thisRobot.y(), data.robots[i].x() - thisRobot.x()) + thisRobot.theta()* 3.14159/180;
				SensorData neighborMeasurement = SensorData(distance, angle);
				//cout << neighborMeasurement.distance << " " << neighborMeasurement.theta << endl;
				output.push_back(neighborMeasurement);
			}
		}
	}
	return output;
}

SensorData SimulateGoalSensor(int inputID, FieldData data, int goalID) {
	//cout << "Simulating sensor" << endl;

	SensorData output = SensorData(0,0);
	
	float pixelsPerCm = (getEntity(202, data).x() - getEntity(200,data).x())/231.14;
	//cout << "Px/Cm: " << pixelsPerCmNew << endl;
	//float pixelsPerCm = 1;
	Robot goal = getRobot(goalID,data);


	pair<float,float>goalPosition(goal.x()/pixelsPerCm,goal.y()/pixelsPerCm);

	Robot thisRobot = getRobot(inputID, data);

	float distance = sqrt(pow(goalPosition.first - thisRobot.x()/pixelsPerCm,2) + pow(goalPosition.second - thisRobot.y()/pixelsPerCm,2));

	float angle = fmod(atan2(goalPosition.second - thisRobot.y()/pixelsPerCm, goalPosition.first - thisRobot.x()/pixelsPerCm)*180/3.14159 + thisRobot.theta(),360);

	if (goal.id() != -1){
		output = SensorData(distance, angle*3.14159/180);
	}

	//cout << "Goal Position: " << goalPosition.first << " " << goalPosition.second << endl;
	//cout << "Goal Vector: " << output.distance << " " << output.theta << endl;
	return output;
}

// Get the sensor readings for a specific robot ID
Reading GetRobotSensorReading(int inputID, vector<Reading> readings) {
	for (unsigned i = 0; i < readings.size(); i++) {
		if (readings[i].id == inputID) {
			return readings[i];
		}
	}
	return readings[0];
}

#endif