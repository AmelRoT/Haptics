struct Pi {

    dsfloat error; 
    dsfloat error_prev; 
    dsfloat y_prev;
    dsfloat y_out; 
    dsfloat K_i; 
    dsfloat K_p; 
    dsfloat T; 
    dsfloat upperLimit; 
    dsfloat lowerLimit; 
    

}; 


#define defaultValue {0,0,0,0,0,0,\
                     0,0,0} 

void pi_calc(struct Pi *pi); 



