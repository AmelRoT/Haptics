#include "pid.h"

void pid_calc(struct PID *pid){

    pid->C1 = pid->T*pid->g*2+4; 
    
    pid->y_n = ((pid->T*pid->g*2-4)/(pid->C1))*pid->y_n_2 + (8/(pid->C1))*pid->y_n_1 + 
    (((pid->Ki*pid->T*pid->T-2*pid->T*pid->Kp+pid->Kd*4)*pid->g-2*pid->Ki*pid->T+4*pid->Kp)/(pid->C1))*pid->error_n_2 +  
    (((2*pid->Ki*pid->T*pid->T-8*pid->Kd)*pid->g-8*pid->Kp)/(pid->C1))*pid->error_n_1 + 
    (((pid->Ki*pid->T*pid->T+2*pid->T*pid->Kp+pid->Kd*4)*pid->g+2*pid->Ki*pid->T+4*pid->Kp)/(pid->C1))*pid->error_n; 

    if(pid->y_n  > pid->upperLimit){
        pid->y_n = pid->upperLimit;
    }

    if(pid->y_n  < pid->lowerLimit){
        pid->y_n = pid->lowerLimit; 
    }

    pid->error_n_2 = pid->error_n_1; 
    pid->error_n_1 = pid->error_n; 
    pid->y_n_2 = pid->y_n_1; 
    pid->y_n_1 = pid->y_n; 

}



