#ifndef SENSORDATA_H
#define SENSORDATA_H

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

#endif