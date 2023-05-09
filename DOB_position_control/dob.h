struct dob {

    dsfloat T_n; 
    dsfloat T_n_1; 
    dsfloat i_n; 
    dsfloat i_n_1;
    dsfloat w_n; 
    dsfloat w_n_1;    
    dsfloat C1; 
    dsfloat g; 
    dsfloat T; 
    dsfloat J; 
    dsfloat Kt; 
}; 

void calc_DOB(struct dob *DOB); 


/// @param T_n -  current estimated disturbance at n 
/// @param T_n_1  - previous value of esitamated disturbance n-1
/// @param i_n -   current at n  
/// @param i_n_1 - current at n-1 
/// @param w_n -   omega/theat at n 
/// @param w_n_1 - omega/theta at n-1
/// @param C1 -  constant for easier writing calc_DOB 
/// @param g - low pass filter value 
/// @param T - Period
/// @param J - intertia
/// @param Kt - current constant for torque 

