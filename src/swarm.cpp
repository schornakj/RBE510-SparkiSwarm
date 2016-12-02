#include <iostream>
#include <string>
#include <cmath>
#include "rbe510.hpp"

using namespace std;

struct SensorData {
	float distance;
	float theta;
};

class RobotAgent {
	// Represents an individual robot.
public:
	int id;
	FieldComputer fc;

	RobotAgent(int inputID, FieldComputer inputFieldComputer) {
		this->id = inputID;
		this->fc = inputFieldComputer;
	}

	Robot getRobot(int id, FieldComputer fc){
    	FieldData data = fc.getFieldData();
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

	vector<SensorData> SimulateSensor() {
		// Use field data derived from the server's computer vision algorithm to measure the distance to neighboring robots.
		vector<SensorData> output;

		FieldData data = fc.getFieldData();

		Robot thisRobot = getRobot(id, fc);

    	for(unsigned i = 0; i < data.robots.size(); i++){
        	if(data.robots[i].id() != id) {
            	output.push_back(SensorData(sqrt(pow(data.robots[i].x() - thisRobot.x(),2) + pow(data.robots[i].y() - thisRobot.y(),2)), data.robots[i].theta()));
        	}
    	}
    	return output;
    }

	void DoFlockingController() {
		vector<SensorData> neighborRangeData = SimulateSensor();
		// TODO: Use fake sensor data to figure out how the robot should move to stay in formation

		float leftWheelSpeed = 0;
		float rightWheelSpeed = 0;
		// TODO: Update Sparki code to allow differential drive
		fc.differentialDrive(id, leftWheelSpeed, rightWheelSpeed);
	}
};

int main(int argc, char *argv[])
{
	string ip = string("127.0.0.1");
	FieldComputer fc(ip);
	fc.enableVerbose();

	FieldData data = fc.getFieldData();

	vector<RobotAgent> robotAgents;
	for(unsigned i = 0; i < data.robots.size(); i++){
        robotAgents.push_back(RobotAgent(data.robots[i].id(), fc));
    }

    while(true) {
    	// Have each robot update itself using swarm algorithm
    	for(unsigned i = 0; i < robotAgents.size(); i++){
        	robotAgents[i].DoFlockingController();
    	}
    }
	return 0;
}