#include "dob_theta.h"


void calc_DOB_theta_minus(struct dob_theta *DOB){

    DOB->C1 = DOB->T*DOB->T*DOB->g2+2*DOB->T*DOB->g1+4; 
    DOB->T_n = ((2*DOB->T*DOB->g1-4-DOB->T*DOB->T*DOB->g2)/(DOB->C1))*DOB->T_n_2 + ((8-DOB->T*DOB->T*DOB->g2*2)/(DOB->C1))*DOB->T_n_1+
    ((DOB->Kt*DOB->T*DOB->T*DOB->g2)/(DOB->C1))*(DOB->i1_n_2-DOB->i2_n_2+2*(DOB->i1_n_1-DOB->i2_n_1)+DOB->i1_n-DOB->i2_n) + 
    ((4*DOB->g2)/(DOB->C1))*(DOB->th2_n_2-DOB->th1_n_2+2*(DOB->th1_n_1-DOB->th2_n_1)+DOB->th2_n-DOB->th1_n); 

    DOB->T_n_2 = DOB->T_n_1; 
    DOB->T_n_1 = DOB->T_n; 

    DOB->th2_n_2 = DOB->th2_n_1; 
    DOB->th2_n_1 = DOB->th2_n; 
    
    DOB->th1_n_2 = DOB->th1_n_1; 
    DOB->th1_n_1 = DOB->th1_n;

    DOB->i1_n_2 = DOB->i1_n_1; 
    DOB->i1_n_1 = DOB->i1_n;

    DOB->i2_n_2 = DOB->i2_n_1; 
    DOB->i2_n_1 = DOB->i2_n;

}
void calc_DOB_theta_plus (struct dob_theta *DOB){

    DOB->C1 = DOB->T*DOB->T*DOB->g2+2*DOB->T*DOB->g1+4; 
    DOB->T_n = ((2*DOB->T*DOB->g1-4-DOB->T*DOB->T*DOB->g2)/(DOB->C1))*DOB->T_n_2 + ((8-DOB->T*DOB->T*DOB->g2*2)/(DOB->C1))*DOB->T_n_1+
    ((DOB->Kt*DOB->T*DOB->T*DOB->g2)/(DOB->C1))*(DOB->i1_n_2+DOB->i2_n_2+2*(DOB->i1_n_1+DOB->i2_n_1)+DOB->i1_n+DOB->i2_n) + 
    ((4*DOB->g2)/(DOB->C1))*(-DOB->th2_n_2-DOB->th1_n_2+2*(DOB->th1_n_1+DOB->th2_n_1)-DOB->th2_n-DOB->th1_n); 

    DOB->T_n_2 = DOB->T_n_1; 
    DOB->T_n_1 = DOB->T_n; 

    DOB->th2_n_2 = DOB->th2_n_1; 
    DOB->th2_n_1 = DOB->th2_n; 
    
    DOB->th1_n_2 = DOB->th1_n_1; 
    DOB->th1_n_1 = DOB->th1_n;

    DOB->i1_n_2 = DOB->i1_n_1; 
    DOB->i1_n_1 = DOB->i1_n;

    DOB->i2_n_2 = DOB->i2_n_1; 
    DOB->i2_n_1 = DOB->i2_n;


}
