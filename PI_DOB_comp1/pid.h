struct PID {

    dsfloat error_n; 
    dsfloat error_n_1;
    dsfloat error_n_2;  
    dsfloat y_n_2; 
    dsfloat y_n_1;
    dsfloat y_n; 
    dsfloat Ki; 
    dsfloat Kp;
    dsfloat Kd;  
    dsfloat T; 
    dsfloat g;  
    dsfloat upperLimit; 
    dsfloat lowerLimit; 
    dsfloat C1; 
}; 


#define defaultValue {0,0,0,0,0,0,\
                     0,0,0,0,0,0,0,0} 


#define defaultValuePID {0,0,0,0,0,0,\
                     20,1,1,1e-5,100,3.1,-3.1,0} 



void pid_calc(struct PID *pid); 



/// @param error_n ->
/// @param error_n_1 ->
/// @param error_n_2 ->
/// @param y_n_2 ->
/// @param y_n_1 ->
/// @param y_n ->
/// @param Ki ->
/// @param Kp ->
/// @param Kd ->
/// @param T ->
/// @param g ->
/// @param upperLimit ->
/// @param lowerLimit ->