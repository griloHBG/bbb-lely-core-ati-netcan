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
        
        //antigo errado, mas funcional
        //controlSignal = (2 * (referenceTorque - currentTorque) + prevActionSignal * (2 * By - Ky * dt)) / (2 * By + Ky * dt);
        
        //novo, usando tustin, mas nao funcional: angularCorrection fica quase constante > 1.3 rad
//        controlSignal = (4*By*prevActionSignal + (Ky*dt - 2*By)*prevprevActionSignal + 2*dt*(currentTorque - prevTorque)) / (2*By + Ky*dt);
//        controlSignal = (4*By*prevActionSignal + (Ky*dt - 2*By)*prevprevActionSignal + 2*dt*(-currentTorque + prevTorque)) / (2*By + Ky*dt);
        
        //novo, usando euler, funcional, baraio
        controlSignal = (2*By*prevActionSignal + (Ky*dt)*prevprevActionSignal + 2*dt*(-currentTorque)) / (2*By + Ky*dt);
    
        prevprevActionSignal = prevActionSignal;
        prevActionSignal = controlSignal;
        
        prevTorque = currentTorque;
        
        return controlSignal;
    }
    
public:
    
    
    const float& Ky;
    const float& By;
    float prevActionSignal = 0; // q[k-1] rad
    float prevprevActionSignal = 0; // q[k-2] rad
    float prevTorque = 0; // tau[k-1] Nm
    float referenceTorque = 0;
    float lastVelocity = 0;
    float lastVelocityFiltered = 0;
    float J = 0;
    float cutoffFreq = 1 * 2 * 3.141596f; // for low pass filter on velocity
};


#endif //LELY_BBB_INTERACTIONCONTROLLER_H
