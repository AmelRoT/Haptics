struct dob_vel {

    dsfloat T_n; 
    dsfloat T_n_1; 
    dsfloat i1_n; 
    dsfloat i1_n_1;
    dsfloat i2_n; 
    dsfloat i2_n_1;
    dsfloat w1_n; 
    dsfloat w1_n_1;    
    dsfloat w2_n; 
    dsfloat w2_n_1; 
    dsfloat g; 
    dsfloat T; 
    dsfloat J; 
    dsfloat Kt; 
}; 

#define defaultValueDOBvel {0,0,0,0,0,0,\
                     0,0,0,0,100,1e-5,0.0001,0.8} 



void calc_DOB_vel_minus(struct dob_vel *DOB); 
void calc_DOB_vel_plus (struct dob_vel *DOB); 


/// @param T_n -  current estimated disturbance at n 
/// @param T_n_1  - previous value of esitamated disturbance n-1
/// @param i_n -   current at n  
/// @param i_n_1 - current at n-1 
/// @param w_n -   omega/theat at n 
/// @param w_n_1 - omega/theta at n-1
/// @param g - low pass filter value 
/// @param T - Period
/// @param J - intertia
/// @param Kt - current constant for torque 

