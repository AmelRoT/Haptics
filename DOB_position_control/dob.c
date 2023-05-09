#include "dob.h"


void calc_DOB(struct dob *DOB){

    DOB->C1 = (2*DOB->J*DOB->g)/(2+DOB->g*DOB->T); 
    DOB->T_n = ((2-DOB->g*DOB->T)/(2+DOB->g*DOB->T))*DOB->T_n_1 + ((DOB->Kt*DOB->g*DOB->T)/(2+DOB->g*DOB->T))*(DOB->i_n+DOB->i_n_1)+DOB->C1*(DOB->w_n_1-DOB->w_n); 
    DOB->T_n_1 = DOB->T_n; 
    DOB->i_n_1 = DOB->i_n; 
    DOB->w_n_1 = DOB->w_n; 
    
}

