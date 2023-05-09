void sigmaCalculate3DOB(struct sigma_motion_3DOB_force *sig ,struct sigma_dob *dob_sigma, struct dob *dob1){

    sig->sigma_mot = (sig->v1_mes-sig->v1_ref)+sig->C1*(sig->theta1_mes-sig->theta1_ref); 

    sig->f_env = dob1->T_n; 

    if(sig->sigma_mot > sig->saturation_sigma){
        sig->sigma_mot = sig->saturation_sigma; 
    }

    if(sig->sigma_mot < -sig->saturation_sigma){
        sig->sigma_mot = -sig->saturation_sigma; 
    }

    //sig->i_con1 = -sig->sigma_mot*sig->K_p_sigma; 
    
    sig->i_con1 = -(sig->sigma_mot+sig->f_env)*sig->K_p_sigma; 


    dob_sigma->i_n = sig->i_con1; 
    dob_sigma->sigma_n = sig->sigma_mot; 
    calc_sigma_DOB(dob_sigma); 


    dob1->i_n = sig->i1_ref; 
    dob1->w_n = sig->v1_mes; 
    calc_DOB(dob1); 

 //  sig->i1_ref = sig->i_con1 + (dob_sigma->J/dob_sigma->Kt*dob_sigma->T_n)+dob1->T_n/dob1->Kt;

    sig->i1_ref = sig->i_con1; 

	if( sig->i1_ref > 3.1){
		 sig->i1_ref = 3.1; 
	}

	if( sig->i1_ref  < -3.1){
		
		 sig->i1_ref = -3.1; 
	}

	if(fabs(sig->i1_ref  ) <= 0.015 ){  // to nullify the zero reference current. 
		sig->i1_ref = 0.0;
	}
    //LM motor


}