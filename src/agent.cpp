#include "agent.h"

using namespace std;

float Agent::GeneralizedLennardJones(float f_Distance){
    float fNormDistExp = pow(TargetDistance/f_Distance,Exponent);
    return -Gain/f_Distance*(fNormDistExp*fNormDistExp-fNormDistExp);
}

//SVector Agent::VectorToGoal(float f_Distance, float f_Angle){
//    
//}

SensorData Agent::FlockingVector(){
    if (!m_tReadings.empty()) {
        SensorData SAccum; // note : create a default constructor in SensorData.h
        for (TVecData::iterator it=m_tReadings.begin(); it=!m_tReadings.end(); ++it) {
            float fTemp;
            if (m_tReadings.distance<1.8*TargetDistance) {
                fTemp=GeneralizedLennardJones(m_tReadings.distance);
            }
            SAccum=AddVectors(SAccum,);
        }
    }
    
}

WheelSpeeds Agent::SpeedFromVector(SVector s_vector){
    
}

WheelSpeeds Agent::ControlStep(Reading s_readings){
    m_tVectorToGoal=s_readings.goalData;
    m_tReadings=s_readings.robotData;
    return WheelSpeedsFromVector(AddVectors(m_tVectorToGoal,FlockingVector()));
}


