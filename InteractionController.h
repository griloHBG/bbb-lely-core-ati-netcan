//
// Created by grilo on 23/02/2021.
//

#ifndef LELY_BBB_INTERACTIONCONTROLLER_H
#define LELY_BBB_INTERACTIONCONTROLLER_H


class InteractionController {
public:
    InteractionController(float Ky, float By, float referenceTorque = 0) : Ky(Ky), By(By), referenceTorque(referenceTorque) {
    
    }
    
    float getControlSignal(float currentTorque, float dt) {
        //y_a = (2 * (tau_d - tau_env) + q_a_ant * (2 * By - Ky * dt)) / (2 * By + Ky * dt)
        float controlSignal;
        
        controlSignal = (2 * (referenceTorque - currentTorque) + lastActionSignal * (2 * By - Ky * dt)) / (2 * By + Ky * dt);
    
        lastActionSignal = controlSignal;
        
        return controlSignal;
    }
    
public:
    float Ky;
    float By;
    float lastActionSignal; // qa
    float referenceTorque;
};


#endif //LELY_BBB_INTERACTIONCONTROLLER_H
