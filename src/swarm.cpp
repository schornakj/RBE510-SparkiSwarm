#include <iostream>
#include <string>
#include <cmath>
#include <ctime>
#include "sensor.h"
#include "sensordata.h"
#include "rbe510.h"
#include "agent.h"

using namespace std;

int main(int argc, char *argv[])
{
	string ip = string("127.0.0.1");
	FieldComputer fc(ip);
	//fc.enableVerbose();
	fc.disableVerbose();
	
	float sensorThreshold = 50;

	//int goalID = 111;
	int goalID = 1;


	FieldData data = fc.getFieldData();

	//double runtime = 5*CLOCKS_PER_SEC; // 10 seconds in clock ticks

	//clock_t start;
	//start = clock();

	//pair<float,float>goalPosition(50,50);
	int count = 0;

    while(true) {
		data = fc.getFieldData();
		
		//SensorData goalData(300,300);
		
		// update robot sensor readings
		cout << "Update # " << count << endl << endl;;

    	for(unsigned i = 0; i < data.robots.size(); i++){
			// for each robot, get its ID and use the ID to simulate its sensor readings
    		cout << "Robot # " << data.robots[i].id() << endl;

			vector<SensorData> currentSensor = SimulateSensor(data.robots[i].id(), data, sensorThreshold);
			Reading currentReading(data.robots[i].id(), currentSensor, SimulateGoalSensor(data.robots[i].id(), data, goalID));

			
			cout << "Pixel Position: " << data.robots[i].x() <<'\t' << data.robots[i].y()<<endl;
			cout << "Heading: " << data.robots[i].theta() << endl;

			
			//cout << currentSensor[0].distance << endl;

			
			//for (vector<SensorData>::iterator j = currentReading.robotData.begin(); j != currentReading.robotData.end(); ++j) {
			//	cout << j->distance << " " << j->theta*180/PI << endl;
			//}
			


			// TODO: Have each robot update itself using swarm algorithm
			Agent a;		
			WheelSpeeds currentSpeeds = a.ControlStep(currentReading);
			
			fc.arcadeDrive(data.robots[i].id(), currentSpeeds.first, currentSpeeds.second);

			//if ((clock() - start)/CLOCKS_PER_SEC >= runtime) {
				//break;
			//}
			cout << endl;
			count++;
    	}
    	usleep(10000);	
    	cout << endl << endl;
    }
    /*
    for(unsigned i = 0; i < data.robots.size(); i++){
    	fc.arcadeDrive(data.robots[i].id(), 0, 0);
    }
    */

	return 0;
}