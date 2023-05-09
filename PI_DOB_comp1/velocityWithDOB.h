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

void velocityTrack(struct dob *dob1,struct velocityTrack *vel1); 
void positionTrack(struct dob *dob1,struct velocityTrack *vel1); 
void syncDOBposition(struct dob *dob1, struct dob *dob2, struct syncWithDobb *syncing); 




/// @param erorr - velocity error 
/// @param K_p   - proportional value for the controller 
/// @param i_con - combined with T_dis to form I_ref 
/// @param theta_mes - measured angle theta 
/// @param theta_ref - reference value of the angle 



