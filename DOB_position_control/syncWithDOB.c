//#include "velocityWithDOB.h"
// already used in other files 

void syncDOBposition(struct dob *dob1, struct dob *dob2, struct syncWithDobb *syncing){

	//
	//   e_1 = theta_motor2 - theta_motor1
	// 
	syncing->error1 = syncing->theta_mes2-syncing->theta_mes1; 
	syncing->i_con1 = syncing->K_p1*syncing-> error1; 
	
	dob1->i_n = syncing->i_ref1; 
    dob1->w_n = syncing->theta_mes1; 
    
	calc_DOB(dob1); 

    syncing->i_ref1 = 0.5*(syncing->i_con1+syncing->i_con2); 

	if(syncing->i_ref1 > 3.1){
		syncing->i_ref1  = 3.1; 
	}

	if(syncing->i_ref1  < -3.1){
		
		syncing->i_ref1 = -3.1; 
	}

	if(fabs(syncing->i_ref1  ) <= 0.05 ){  // to nullify the zero reference current. 
		syncing->i_ref1   = 0.0;
	}

	//
	//   e_2 = 2* Theta_ref - theta_motor1 - theta_motor2
	// 

	syncing->error2 = -syncing->theta_mes2-syncing->theta_mes1+2*syncing->theta_ref; 
	syncing->i_con2 = syncing->K_p2*syncing-> error2; 
	
	dob2->i_n = syncing->i_ref2; 
    dob2->w_n = syncing->theta_mes2; 
    
	calc_DOB(dob2); 

    syncing->i_ref2 = 0.5*(syncing->i_con2-syncing->i_con1); 

	if(syncing->i_ref2 > 3.1){
		syncing->i_ref2  = 3.1; 
	}

	if(syncing->i_ref2  < -3.1){
		syncing->i_ref2 = -3.1; 
	}

	if(fabs(syncing->i_ref2  ) <= 0.05 ){  // to nullify the zero reference current. 
		syncing->i_ref2   = 0.0;
	}


}




