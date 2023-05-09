#include "dob_vel.h"


void calc_DOB_vel_minus(struct dob_vel *DOB){

    DOB->T_n = ((2-DOB->g*DOB->T)/(2+DOB->g*DOB->T))*DOB->T_n_1 + ((DOB->g*DOB->T*DOB->Kt)/(DOB->J*(2+DOB->g*DOB->T)))* 
    (DOB->i1_n-DOB->i2_n+DOB->i1_n_1-DOB->i2_n_1)+((2*DOB->g)/(2+DOB->g*DOB->T))*(DOB->w2_n-DOB->w1_n+DOB->w1_n_1-DOB->w2_n_1); 

    DOB->T_n_1  = DOB->T_n; 
    DOB->i1_n_1 = DOB->i1_n; 
    DOB->i2_n_1 = DOB->i2_n; 
    DOB->w1_n_1 = DOB->w1_n; 
    DOB->w2_n_1 = DOB->w2_n; 

}


void calc_DOB_vel_plus (struct dob_vel *DOB){


    DOB->T_n = ((2-DOB->g*DOB->T)/(2+DOB->g*DOB->T))*DOB->T_n_1 + ((DOB->g*DOB->T*DOB->Kt)/(DOB->J*(2+DOB->g*DOB->T)))*
    (DOB->i1_n+DOB->i2_n+DOB->i1_n_1+DOB->i2_n_1)+((2*DOB->g)/(2+DOB->g*DOB->T))*(-DOB->w2_n-DOB->w1_n+DOB->w1_n_1+DOB->w2_n_1); 

    DOB->T_n_1  = DOB->T_n; 
    DOB->i1_n_1 = DOB->i1_n; 
    DOB->i2_n_1 = DOB->i2_n; 
    DOB->w1_n_1 = DOB->w1_n; 
    DOB->w2_n_1 = DOB->w2_n; 

}

