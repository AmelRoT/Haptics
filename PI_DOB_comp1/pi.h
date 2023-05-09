struct Pi {

    dsfloat error; 
    dsfloat error_prev; 
    dsfloat y_prev;
    dsfloat y_out; 
    dsfloat Ki; 
    dsfloat Kp; 
    dsfloat T; 
    dsfloat upperLimit; 
    dsfloat lowerLimit; 
}; 


#define defaultValue {0,0,0,0,0,0,\
                     0,0,0} 

#define defaultSyncVel {0,0,0,0,-10,-1,1e-5,3.1,-3.1}

void pi_calc(struct Pi *pi); 



