struct velocityTrack{

    dsfloat error; 
    dsfloat i_con; 
    dsfloat K_p; 
    dsfloat v_mes; 
    dsfloat v_ref; 
    dsfloat i_ref; 
    dsfloat theta_mes; 
    dsfloat theta_ref; 
};


struct syncWithDobb {

    dsfloat error1; 
    dsfloat i_con1; 
    dsfloat K_p1; 
    dsfloat v_mes1; 
    dsfloat v_ref1; 
    dsfloat i_ref1; 
    dsfloat theta_mes1; 
    dsfloat error2; 
    dsfloat i_con2; 
    dsfloat K_p2; 
    dsfloat v_mes2; 
    dsfloat v_ref2; 
    dsfloat i_ref2; 
    dsfloat theta_mes2; 
    dsfloat theta_ref; 

};



struct sigma_motion_1DOB {

    dsfloat sigma_mot; 
    dsfloat v1_mes; 
    dsfloat v1_ref; 
    dsfloat C1;
    dsfloat theta1_mes; 
    dsfloat theta1_ref; 
    dsfloat i_con1; 
    dsfloat K_p_sigma; 
    dsfloat i1_ref; 
}; 


struct sigma_motion_2DOB{

    dsfloat sigma_mot; 
    dsfloat v1_mes; 
    dsfloat v1_ref; 
    dsfloat C1;
    dsfloat theta1_mes; 
    dsfloat theta1_ref; 
    dsfloat i_con1; 
    dsfloat K_p_sigma; 
    dsfloat i1_ref; 
}; 

struct sigma_motion_3DOB_force {

    dsfloat sigma_mot; 
    dsfloat v1_mes; 
    dsfloat v1_ref; 
    dsfloat C1;
    dsfloat theta1_mes; 
    dsfloat theta1_ref; 
    dsfloat i_con1; 
    dsfloat K_p_sigma; 
    dsfloat i1_ref; 
    dsfloat f_env; 
    dsfloat saturation_sigma; 

}; 

struct sigma_mf_master_slave{

    dsfloat sigma_mot; 
    dsfloat v1_mes; 
    dsfloat theta_1; 
    dsfloat v2_mes; 
    dsfloat theta_2; 
    dsfloat C1;
    dsfloat C2; 
    dsfloat K1;
    dsfloat K2;  
    dsfloat force_m; 
    dsfloat force_s; 
    dsfloat G1; // scalling fator 
    dsfloat G11; // scalling fator 
    dsfloat sat_sigma; 
    dsfloat i_ref;
    dsfloat i_con; 
    dsfloat K_p_sigma;
 
}; 



void velocityTrack(struct dob *dob1,struct velocityTrack *vel1); 
void positionTrack(struct dob *dob1,struct velocityTrack *vel1); 
void syncDOBposition(struct dob *dob1, struct dob *dob2, struct syncWithDobb *syncing); 
// void sigmaCalculate(struct sigma_motion_1DOB *sig ,struct sigma_dob *dob_sigma);

#define sigma_motion_3DOB_force_defaults{0,0,0,\
0,0,0,0,0,0,0,0}

#define sigma_motionDefaults { 0,0,0,\
    0,0,0,0,0,0}

#define sigma_mf_master_slave_defaults {0,0,0,\
0,0,0,0,0,0,0,0,0,0,0,0}

/// @param erorr - velocity error 
/// @param K_p   - proportional value for the controller 
/// @param i_con - combined with T_dis to form I_ref 
/// @param theta_mes - measured angle theta 
/// @param theta_ref - reference value of the angle 



