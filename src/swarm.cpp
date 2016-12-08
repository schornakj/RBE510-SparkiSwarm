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

    while(true) {
		data = fc.getFieldData();
		robotFakeSensors.clear();
		
		SensorData goalData(0,0);
		
		// update robot sensor readings
    	for(unsigned i = 0; i < data.robots.size(); i++){
			// TODO: for each robot, get its ID and use the ID to simulate its sensor readings
        	robotFakeSensors.push_back(Reading(data.robots[i].id(), SimulateSensor(data.robots[i].id(), data, sensorThreshold)), goalData);
    	}
				
    	// TODO: Have each robot update itself using swarm algorithm		
    }
	return 0;
}