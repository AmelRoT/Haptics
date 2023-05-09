#include "sync.h"

void calcSyncPosition(struct Pi *pi1, struct Pi *pi2, struct sync *syncVar){

	//
	//   e_1 = theta_motor2 - theta_motor1
	// 

    pi1->error = syncVar->theta_m2-syncVar->theta_m1; 
    pi_calc(pi1); 
    syncVar->i1_ref = 0.5*(pi1->y_out+pi2->y_out); 

	if(syncVar->i1_ref > 3.1){
		syncVar->i1_ref  = 3.1; 
	}

	if(syncVar->i1_ref  < -3.1){
		syncVar->i1_ref = -3.1; 
	}

	if(fabs(syncVar->i1_ref ) <= 0.05){   // to nullify the zero reference current. 
		syncVar->i1_ref  = 0.0;
	}

	//
	//   e_2 = 2* Theta_ref - theta_motor1 - theta_motor2
	// 

	pi2->error = -syncVar->theta_m1-syncVar->theta_m2+2*syncVar->theta_ref; 
	pi_calc(pi2); 
	syncVar->i2_ref = 0.5*(pi2->y_out-pi1->y_out); 

	if(syncVar->i2_ref > 3.1){
		syncVar->i2_ref = 3.1; 
	}

	if(syncVar->i2_ref  < -3.1){
		syncVar->i2_ref  = -3.1; 
	}

	if(fabs(syncVar->i2_ref ) <= 0.05){  // to nullify the zero reference current. 
		syncVar->i2_ref  = 0.0;
	}


}
