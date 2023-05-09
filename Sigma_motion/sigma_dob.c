#include "sigma_dob.h"

void calc_sigma_DOB(struct sigma_dob *DOB){

    DOB->C1 = (2*DOB->g)/(2+DOB->g*DOB->T); 
    DOB->T_n = ((2-DOB->g*DOB->T)/(2+DOB->g*DOB->T))*DOB->T_n_1 + ((DOB->Kt*DOB->g*DOB->T)/(DOB->J*(2+DOB->g*DOB->T)))*(DOB->i_n+DOB->i_n_1)+DOB->C1*(DOB->sigma_n_1-DOB->sigma_n); 
    DOB->T_n_1 = DOB->T_n; 
    DOB->i_n_1 = DOB->i_n; 
    DOB->sigma_n_1 = DOB->sigma_n; 
}



