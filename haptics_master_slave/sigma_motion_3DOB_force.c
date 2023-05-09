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

   // sig->i1_ref = sig->i_con1 + (dob_sigma->J/dob_sigma->Kt*dob_sigma->T_n)+dob1->T_n/dob1->Kt;

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


}



void calcMasterSlaveInt(struct sigma_mf_master_slave *sig ,struct sigma_dob *dob_sigma, struct dob *dob1){

    sig->sigma_mot = sig->C2*(sig->v1_mes-sig->v2_mes)+sig->C1*(sig->theta_1-sig->K1*sig->theta_2)+sig->G1*sig->force_s;

    sig->force_m = dob1->T_n; 
    // slave will be taken from calculating the slave value -> in the same manner the master will be taken 

    if(sig->sigma_mot > sig->sat_sigma){
        sig->sigma_mot = sig->sat_sigma; 
    } 

    if(sig->sigma_mot < -sig->sat_sigma){
        sig->sigma_mot = -sig->sat_sigma; 
    }

    //sig->i_con = -sig->sigma_mot*sig->K_p_sigma; 
    
    sig->i_con = -(sig->sigma_mot+sig->G11*sig->force_m)*sig->K_p_sigma; 


    dob_sigma->i_n = sig->i_con; 
    dob_sigma->sigma_n = sig->sigma_mot; 
    calc_sigma_DOB(dob_sigma); 

    dob1->i_n = sig->i_ref; 
    dob1->w_n = sig->v1_mes; 
    calc_DOB(dob1); 

   // sig->i1_ref = sig->i_con + (dob_sigma->J/dob_sigma->Kt*dob_sigma->T_n)+dob1->T_n/dob1->Kt;

    sig->i_ref = sig->i_con; 

	if( sig->i_ref > 3.1){
		 sig->i_ref = 3.1; 
	}

	if( sig->i_ref  < -3.1){
		
		 sig->i_ref = -3.1; 
	}

	if(fabs(sig->i_ref  ) <= 0.015 ){  // to nullify the zero reference current. 
		sig->i_ref = 0.0;
	}

}


void calcMasterSlaveInt2(struct sigma_mf_master_slave *sig ,struct sigma_dob *dob_sigma, struct dob *dob1){

    sig->sigma_mot = sig->C2*(-sig->v1_mes+sig->K2*sig->v2_mes)+sig->C1*(-sig->theta_1+sig->K1*sig->theta_2)+sig->G11*sig->force_m;

    sig->force_s = dob1->T_n; 
    // slave will be taken from calculating the slave value -> in the same manner the master will be taken 

    if(sig->sigma_mot > sig->sat_sigma){
        sig->sigma_mot = sig->sat_sigma; 
    } 

    if(sig->sigma_mot < -sig->sat_sigma){
        sig->sigma_mot = -sig->sat_sigma; 
    }

    //sig->i_con = -sig->sigma_mot*sig->K_p_sigma; 
    
    sig->i_con = -(sig->sigma_mot+sig->G1*sig->force_s)*sig->K_p_sigma; 


    dob_sigma->i_n = sig->i_con; 
    dob_sigma->sigma_n = sig->sigma_mot; 
    calc_sigma_DOB(dob_sigma); 


    dob1->i_n = sig->i_ref; 
    dob1->w_n = sig->v2_mes; 
    calc_DOB(dob1); 

   // sig->i1_ref = sig->i_con + (dob_sigma->J/dob_sigma->Kt*dob_sigma->T_n)+dob1->T_n/dob1->Kt;

    sig->i_ref = sig->i_con; 

	if( sig->i_ref > 3.1){
		 sig->i_ref = 3.1; 
	}

	if( sig->i_ref  < -3.1){
		
		 sig->i_ref = -3.1; 
	}

	if(fabs(sig->i_ref  ) <= 0.015 ){  // to nullify the zero reference current. 
		sig->i_ref = 0.0;
	}


}