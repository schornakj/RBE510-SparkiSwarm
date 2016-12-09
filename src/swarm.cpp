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
	
	float sensorThreshold = 500;

	FieldData data = fc.getFieldData();

    while(true) {
		data = fc.getFieldData();
		
		SensorData goalData(0,0);
		
		// update robot sensor readings
    	for(unsigned i = 0; i < data.robots.size(); i++){
			// for each robot, get its ID and use the ID to simulate its sensor readings
			vector<SensorData> currentSensor = SimulateSensor(data.robots[i].id(), data, sensorThreshold);
			Reading currentReading(data.robots[i].id(), currentSensor, goalData);

			cout << "Robot #" << currentReading.id << " sees neighbors at:" << endl;

			
			//cout << currentSensor[0].distance << endl;

			
			for (vector<SensorData>::iterator j = currentReading.robotData.begin(); j != currentReading.robotData.end(); ++j) {
				cout << j->distance << " " << j->theta << endl;
			}
			


			// TODO: Have each robot update itself using swarm algorithm			
			WheelSpeeds currentSpeeds(0,0);
			
			//fc.arcadeDrive(data.robots[i].id(), currentSpeeds.first, currentSpeeds.second);

    	}	
    }
	return 0;
}