#include "agent.h"

using namespace std;

float Agent::GeneralizedLennardJones(float f_Distance){
    float fNormDistExp = pow(TargetDistance/f_Distance,Exponent);
    return -Gain/f_Distance*(fNormDistExp*fNormDistExp-fNormDistExp);
}

SensorData Agent::FlockingVector(){
    if (!m_tReadings.empty()) {
        SensorData SAccum;
        for (TVecData::iterator it=m_tReadings.begin(); it=!m_tReadings.end(); ++it) {
            float fTemp;
            if (it->distance<1.8*TargetDistance) {
                fTemp=GeneralizedLennardJones(it->distance);
            }
            SAccum=AddVectors(SAccum,SensorData(fTemp,it->angle));
        }
        SAccum.distance=SAccum.distance/m_tReadings.size();
        if (SAccum.distance > MaxSpeed) {
            SAccum.distance=MaxSpeed;
        }
        return SAccum;
    }
    else {
        return SensorData();
    }
        
}

WheelSpeeds Agent::SpeedFromVector(SVector s_vector){
    
}

WheelSpeeds Agent::ControlStep(Reading s_readings){
    m_tVectorToGoal=s_readings.goalData;
    m_tReadings=s_readings.robotData;
    return WheelSpeedsFromVector(AddVectors(m_tVectorToGoal,FlockingVector()));
}


