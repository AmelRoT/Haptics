#include "pi.h"

void pi_calc(struct Pi *pi){

    pi->y_out = pi->y_prev + (pi->Ki*pi->T/2 + pi-> Kp)*pi-> error + (pi->Ki*pi->T/2 - pi->Kp)* pi->error_prev; 

    if(pi->y_out  > pi->upperLimit){
        pi->y_out = pi->upperLimit;
    }

    if(pi->y_out  < pi->lowerLimit){
        pi->y_out = pi->lowerLimit; 
    }

    pi->y_prev = pi->y_out; 
    pi->error_prev = pi->error; 
    
}



