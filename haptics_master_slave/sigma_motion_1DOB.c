void sigmaCalculate(struct sigma_motion_1DOB *sig ,struct sigma_dob *dob_sigma){

    sig->sigma_mot = (sig->v1_mes-sig->v1_ref)+sig->C1*(sig->theta1_mes-sig->theta1_ref); 
    sig->i_con1 = -sig->sigma_mot*sig->K_p_sigma; 

    dob_sigma->i_n = sig->i1_ref; 
    dob_sigma->sigma_n = sig->sigma_mot; 
    calc_sigma_DOB(dob_sigma); 

  //  sig->i1_ref = sig->i_con1 + (dob_sigma->J/dob_sigma->Kt*dob_sigma->T_n);
	sig->i1_ref = sig->i_con1; 

	if( sig->i1_ref > 3.1){
		 sig->i1_ref = 3.1; 
	}

	if( sig->i1_ref  < -3.1){
		
		 sig->i1_ref = -3.1; 
	}

	if(fabs(sig->i1_ref  ) <= 0.05 ){  // to nullify the zero reference current. 
		sig->i1_ref = 0.0;
	}



}