#include "friciton_comp.h"

void calculateFriction(struct friciton_comp *friction){

    if(fabs(friction->w_n)< 1e-3){
        friction-> f_n = 0; 
    }
    #if 0 
    
    else if(fabs(friction->w_n)> 1e-3){
        friction->f_n = 0.025*sign(friction->w_n);
    }
    #endif 

    else{
        friction->f_n = 0.025*sign(friction->w_n);
    }

}