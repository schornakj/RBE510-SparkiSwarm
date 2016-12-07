#include <iostream>
#include <string>
#include <cmath>
#include "sensor.h"
#include "sensordata.h"
#include "rbe510.h"
#include "agent.h"

using namespace std;

int main(int argc, char *argv[])
{
	string ip = string("127.0.0.1");
	FieldComputer fc(ip);
	fc.enableVerbose();
	
	float sensorThreshold = 30;
	
	vector<Reading> robotFakeSensors;

	FieldData data = fc.getFieldData();

	vector<Agent> robotAgents;
	for(unsigned i = 0; i < data.robots.size(); i++){
		// TODO: create an Agent for each robot using the Agent constructor
        //robotAgents.push_back(RobotAgent(data.robots[i].id(), fc));
    }

    while(true) {
		data = fc.getFieldData();
		robotFakeSensors.clear();
		
		// update robot sensor readings
    	for(unsigned i = 0; i < robotAgents.size(); i++){
			// TODO: for each Agent, get its ID and use the ID to simulate sensor readings for that robot
        	robotFakeSensors.push_back(Reading(robotAgents[i].id, SimulateSensor(robotAgents[i].id, data, sensorThreshold)));
    	}
		
    	// Have each robot update itself using swarm algorithm		
    }
	return 0;
}