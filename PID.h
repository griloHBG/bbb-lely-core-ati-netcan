//
// Created by grilo on 21/02/2021.
//

#ifndef LELY_BBB_PID_H
#define LELY_BBB_PID_H

#include <array>

class PID {

public:
    
    PID(float kp, float ki, float kd, float referencePosition = 0, float referenceVelocity = 0) : kp(kp), ki(ki), kd(kd), errorPositionSum(0.f), errorPosition(0.f), errorVelocity(0.f), referencePosition(referencePosition), referenceVelocity(referenceVelocity) {}
    
    void setReference(float newPositionReference, float newVelocityReference) {
        referencePosition = newPositionReference;
        referenceVelocity = newVelocityReference;
    }
    
    float getControlSignal(float currentPosition, float currentVelocity, float dt) {
        errorPosition = referencePosition - currentPosition;
        errorVelocity = referenceVelocity - currentVelocity;
        
        errorPositionSum += errorPosition * dt;
        
        float controlSignal = kp * errorPosition + kd * errorVelocity + ki * errorPositionSum;
        
        return controlSignal;
    }
    
public:
    float kp;
    float ki;
    float kd;
    
    float errorPositionSum;
    float errorPosition;
    float errorVelocity;
    float referencePosition;
    float referenceVelocity;
};


#endif //LELY_BBB_PID_H
