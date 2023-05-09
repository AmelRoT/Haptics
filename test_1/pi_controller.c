#include "pi_controller.h"

void pi_calc(struct Pi *pi){

    pi->y_out = pi->y_prev + (pi->K_i*pi->T/2 + pi-> K_p)*pi-> error + (pi->K_i*pi->T/2 - pi->K_p)* pi->error_prev; 

    if(pi->y_out  > pi->upperLimit){
        pi->y_out = pi->upperLimit;
    }

    if(pi->y_out  < pi->lowerLimit){
        pi->y_out = pi->lowerLimit; 
    }

    pi->y_prev = pi->y_out; 
    pi->error_prev = pi->error; 
}



