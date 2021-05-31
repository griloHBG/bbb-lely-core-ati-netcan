//
// Created by grilo on 23/02/2021.
//

#ifndef LELY_BBB_INTERACTIONCONTROLLER_H
#define LELY_BBB_INTERACTIONCONTROLLER_H


class InteractionController {
public:
    InteractionController(float& Ky, float& By, float referenceTorque = 0) : Ky(Ky), By(By), referenceTorque(referenceTorque)
    {
    }
    
    float getControlSignal(float currentTorque, float dt) {
        float controlSignal;
    
        controlSignal = (2 * (referenceTorque - currentTorque) + lastActionSignal * (2 * By - Ky * dt)) / (2 * By + Ky * dt);
    
        lastActionSignal = controlSignal;
        
        return controlSignal;
    }
    
public:
    const float& Ky;
    const float& By;
    float lastActionSignal = 0; // qa
    float referenceTorque = 0;
    float lastVelocity = 0;
    float lastVelocityFiltered = 0;
    float J = 0;
    float cutoffFreq = 1 * 2 * 3.141596f; // for low pass filter on velocity
};


#endif //LELY_BBB_INTERACTIONCONTROLLER_H
