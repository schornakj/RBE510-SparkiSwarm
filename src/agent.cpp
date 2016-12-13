#include "agent.h"


using namespace std;

Agent::Agent(){
    //constructor
}

float Agent::GeneralizedLennardJones(float f_Distance){
    float fNormDistExp = pow(TargetDistance/f_Distance,Exponent);
    float output = -Gain/f_Distance*(fNormDistExp*fNormDistExp-fNormDistExp);
    cout << "LJ " << output << endl;
    return output;
}

SensorData Agent::FlockingVector(){
    if (!m_tReadings.empty()) {
        SensorData SAccum;
        for (TVecData::iterator it=m_tReadings.begin(); it!=m_tReadings.end(); ++it) {
            float fTemp;
            //cout << "Distance: " << it->distance << " vs. " << 2.2*TargetDistance << endl;
            if (it->distance < 1.8*TargetDistance) {
            	fTemp=GeneralizedLennardJones(it->distance);
            	cout << "Theta " << it->theta*180/PI << endl;
            	SAccum=AddVectors(SAccum,SensorData(fTemp,it->theta));

            }
        }
        SAccum.distance=SAccum.distance/m_tReadings.size();
        if (SAccum.distance > MaxSpeed/2) {
            SAccum.distance = MaxSpeed/2;
        }
        else if(SAccum.distance < -MaxSpeed/2) {
            SAccum.distance = -MaxSpeed/2;
        }
        cout << "SAcum " << SAccum.distance << " " << SAccum.theta*180/PI << endl;
        return SAccum;
    }
    else {
        return SensorData();
    }
        
}

WheelSpeeds Agent::SpeedFromVector(SensorData  s_vector){
    // If < 10 degrees difference, don't turn, just go straight
    //cout << "Length: " << s_vector.distance << " Angle: " << s_vector.theta << endl;
    float fSpeed1, fSpeed2 = 0;
    if (abs(s_vector.theta) < SoftTurnThreshold) {
        fSpeed1 = min(s_vector.distance,BaseSpeed);
        fSpeed2 = min(s_vector.distance,BaseSpeed);
    }

    // else if < 70 degrees difference, do a soft turn
    else if (abs(s_vector.theta) < HardTurnThreshold) {
        float fSpeedFactor = (HardTurnThreshold - abs(s_vector.theta))/HardTurnThreshold;
        fSpeed1 = BaseSpeed - BaseSpeed*(1-fSpeedFactor);
        fSpeed2 = BaseSpeed + BaseSpeed*(1-fSpeedFactor);
    }
    // else, do a hard turn in place
    else {
        fSpeed1 = -MaxSpeed;
        fSpeed2 = MaxSpeed;
    }

    float leftSpeed, rightSpeed = 0;
    if (s_vector.theta > 0 && s_vector.theta < PI) {
        leftSpeed = fSpeed2;
        rightSpeed = fSpeed1;
    } else {
        leftSpeed = fSpeed1;
        rightSpeed = fSpeed2;        
    }
    cout << "Left Speed: " << leftSpeed << '\t'<<"Right Speed: " << rightSpeed << endl;
    return pair<float,float>(leftSpeed,rightSpeed);
}

WheelSpeeds Agent::ControlStep(Reading s_readings){
	if (s_readings.goalData.distance < MaxSpeed/2){
    	m_tVectorToGoal=s_readings.goalData;
	}
	else {
		m_tVectorToGoal=SensorData(MaxSpeed/2,s_readings.goalData.theta);
	}
    cout << "Goal: " << s_readings.goalData.distance <<'\t'<<s_readings.goalData.theta << endl;
    m_tReadings=s_readings.robotData;



    SensorData vectorSum = AddVectors(m_tVectorToGoal,FlockingVector());
    //SensorData vectorSum = AddVectors(SensorData(0,0),FlockingVector());

    cout << "FV " << vectorSum.distance << " " << vectorSum.theta << endl;

    WheelSpeeds output = SpeedFromVector(vectorSum);
    
    return output;
}

SensorData Agent::AddVectors(SensorData inputA, SensorData inputB) {
    float xA, xB, xC, yA, yB, yC;
    //cout << "Length A " << inputA.distance << " Length B: " << inputB.distance << endl;
    //cout << "Angle A: " << inputA.theta << " Angle B: " << inputB.theta << endl;
    
    xA = inputA.distance*cos(inputA.theta);
    xB = inputB.distance*cos(inputB.theta);
    
    yA = inputA.distance*sin(inputA.theta);
    yB = inputB.distance*sin(inputB.theta);
    
    xC = xA + xB;
    yC = yA + yB;
    
    return SensorData(sqrt(pow(xC,2)+pow(yC,2)), atan2(yC,xC));
}


