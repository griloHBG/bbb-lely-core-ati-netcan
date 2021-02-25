//
// Created by grilo on 23/02/2021.
//

#ifndef LELY_BBB_INTERACTIONCONTROLLER_H
#define LELY_BBB_INTERACTIONCONTROLLER_H


class InteractionController {
public:
    InteractionController(float Ky, float By, float referenceTorque = 0) : Ky(Ky), By(By), referenceTorque(referenceTorque) {
        float J1 = (1/12.f * (.038f*.038f + .032f*.032f) + .007f*.007f) * .038f*.032f*.025f * 2700;
        float J2 = (1/12.f * (.038f*.038f + .19f*.19f) + .083f*.083f) * .038f*.19f*.004f * 2700;
        float J3 = (.5f * .011f*.011f + .076f*.076f) * 3.141596f * .011f*.011f * 1140;
        J = J1 + J2 + J3;
    }
    
    float getControlSignal(float currentTorque, float dt) {
        float controlSignal;
    
        //y_a = (2 * (tau_d - tau_env) + q_a_ant * (2 * By - Ky * dt)) / (2 * By + Ky * dt)
        controlSignal = (2 * (referenceTorque - currentTorque) + lastActionSignal * (2 * By - Ky * dt)) / (2 * By + Ky * dt);
    
        lastActionSignal = controlSignal;
        
        return controlSignal;
    }
    
    float getTorqueFromObserver(float lastControlSignal, float currentVelocity, float dt) {
        
        float velocityFiltered = ( dt * currentVelocity + cutoffFreq * lastVelocityFiltered ) / (cutoffFreq + dt);
        
        float externalTorque = J * (velocityFiltered - lastVelocityFiltered)/dt - lastControlSignal;
    
        lastVelocityFiltered = velocityFiltered;
        
        return externalTorque;
    }
    
public:
    float Ky = 0;
    float By = 0;
    float lastActionSignal = 0; // qa
    float referenceTorque = 0;
    float lastVelocity = 0;
    float lastVelocityFiltered = 0;
    float J = 0;
    float cutoffFreq = 1 * 2 * 3.141596f; // for low pass filter on velocity
};


#endif //LELY_BBB_INTERACTIONCONTROLLER_H
