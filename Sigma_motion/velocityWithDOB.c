#include "velocityWithDOB.h"

void velocityTrack(struct dob *dob1,struct velocityTrack *vel1){

    vel1->error = vel1->v_ref - vel1->v_mes; 
    vel1->i_con = vel1->K_p*vel1->error; 

    dob1->i_n = vel1->i_ref; 
    dob1->w_n = vel1->v_mes; 

    calc_DOB(dob1); 

    vel1->i_ref = (vel1->i_con+dob1->T_n/dob1->Kt); 

	if(vel1->i_ref > 3.1){
		vel1->i_ref  = 3.1; 
	}

	if(vel1->i_ref  < -3.1){
		vel1->i_ref  = -3.1; 
	}

	if(fabs(vel1->i_ref ) <= 0.05 ){  // to nullify the zero reference current. 
		vel1->i_ref  = 0.0;
	}

}

void positionTrack(struct dob *dob1,struct velocityTrack *vel1){

    vel1->error = vel1->theta_ref - vel1->theta_mes; 
    vel1->i_con = vel1->K_p*vel1->error; 

    dob1->i_n = vel1->i_ref; 
    dob1->w_n = vel1->theta_mes; 

    calc_DOB(dob1); 

 //   vel1->i_ref = (vel1->i_con+dob1->T_n/dob1->Kt); 
	vel1->i_ref = vel1->i_con; 
	
	if(vel1->i_ref > 3.1){
		vel1->i_ref  = 3.1; 
	}

	if(vel1->i_ref  < -3.1){
		vel1->i_ref  = -3.1; 
	}
#if 0 
	if(fabs(vel1->i_ref ) <= 0.05 ){  // to nullify the zero reference current. 
		vel1->i_ref  = 0.0;
	}
#endif 
#if 0 
	if(fabs(vel1->i_ref ) <= 0.001 ){  // to nullify the zero reference current. 
		vel1->i_ref  = 0.0;
	}
#endif 

}


















