struct dob_theta {

    dsfloat T_n; 
    dsfloat T_n_1; 
    dsfloat T_n_2; 
    dsfloat i1_n; 
    dsfloat i1_n_1;
    dsfloat i1_n_2;
    dsfloat i2_n; 
    dsfloat i2_n_1;
    dsfloat i2_n_2;
    dsfloat th1_n; 
    dsfloat th1_n_1;    
    dsfloat th1_n_2;    
    dsfloat th2_n; 
    dsfloat th2_n_1; 
    dsfloat th2_n_2; 
    dsfloat g1;
    dsfloat g2;  
    dsfloat T; 
    dsfloat J; 
    dsfloat Kt; 
    dsfloat C1; 
}; 

#define defaultValueDOBtheta {0,0,0,0,0,0,\
                     0,0,0,0,0,0,0,0,0,10,100,1e-5,0.001,0.8,0} 


void calc_DOB_theta_minus(struct dob_theta *DOB); 
void calc_DOB_theta_plus (struct dob_theta *DOB); 
