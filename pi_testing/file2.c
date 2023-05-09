#include <stdio.h>      /* STandard Digital I/O*/ 
#include <brtenv.h>     /* Basic Real Time ENViroment*/
#include <dstypes.h>	/* Standard Types Aupported by Dspace*/
#include <hostsvc.h>	/* Host Service*/
#include <ds3001.h>     /* Incremental Encoder Interface Board*/
#include <ds2103.h>     /* Multi-Channel D/A Converter Board*/  
#include <ds4002.h>     /* DS4002 Digital I/O Board*/  
#include <ds2003.h>     /* Multi-Channel A/D Converter Board*/
#include <math.h>       /* Math library*/
#include "pi_controller.c"


//#define     DT      0.75e-4          /* 75 us simulation step size */
// #define     DT      0.75e-4         /* 75 us simulation step size */
#define     DT      0.5e-4 

#define     pi      3.141592653589793238462643383279502884197 /* pi value*/

#define ADCCOUNT 5	                /* number of AD channels being used */
#define DACCOUNT 5	                /* number of DA channels being used */
#define GPIOACCOUNT 8	                /* number of DA channels being used */

int dacchannels[5] = {1,2,3,4,5};
int adcchannels[5] = {1,2,3,4,5};
long gpiochannels[8] = {1,2,3,4,5,6,7,8};


dsfloat DAC_data[DACCOUNT] = {0,0,0,0,0}; //array for output DAC data
dsfloat ADC_data[ADCCOUNT];
dsfloat GIO_DATA[GPIOACCOUNT] = {0,0,0,0,0,0,0,0};



//------------------------------------------------------------------------------//
//							Global Variables 									//
//------------------------------------------------------------------------------//

//
// Pi structs 
// 
struct Pi pi1;   
struct Pi pi2; 
struct Pi pi_s1; 
struct Pi pi_s2; 

struct Pi pi_lm1; 
struct Pi pi_lm2; 
struct Pi pi_lm_motor1; 
//
// Gearbox motor 1 - params  
// 

dsfloat error_theta = 0;
dsfloat Kp_theta = 25; 
dsfloat dir = 1; 
dsfloat ki = 100; 
dsfloat kp = 1; 
dsfloat error_track = 0; 
dsfloat vel_track = 0; 
dsfloat v1_motor = 0; 
dsfloat v1_ref = 1.5; 
dsfloat t=0;
dsfloat position1 = 0; 
dsfloat position1_prime= 0;
dsfloat freq1 = 0.0;
dsfloat theta_ref = 2*pi; 
dsfloat theta_m = 0; 

//
// Gearbox motor 2 - params  
// 

dsfloat error_theta2 = 0;
dsfloat Kp_theta2 = 25; 
dsfloat dir2 = 1; 
dsfloat ki2 = 100; 
dsfloat kp2 = 1; 
dsfloat error_track2 = 0; 
dsfloat vel_track2 = 0; 
dsfloat v2_motor = 0; 
dsfloat v2_ref = 1.5;
dsfloat position2 = 0; 
dsfloat position2_prime= 0;
dsfloat freq2 = 0.0;
dsfloat theta_ref2 = 2*pi; 
dsfloat theta_m2 = 0; 


//	
// 	Synchronization variables 
// 

dsfloat i1_ref = 0; 
dsfloat i2_ref = 0; 
dsfloat ki_s1  = 10; 
dsfloat kp_s1  = 1; 
dsfloat ki_s2  = 10; 
dsfloat kp_s2  = 1; 
dsfloat v_ref_s = 1.2; 
dsfloat theta_ref_s = 5; 
//
// Linear motor - params   
//
dsfloat pos_lm1 = 0; 
dsfloat pos_lm2 = 0; 
dsfloat theta_lm1 = 0; 
dsfloat theta_lm2 = 0; 
dsfloat theta_ref_lm = 0.2; 
dsfloat ki_lm1  = 10; 
dsfloat kp_lm1  = 0.2; 
dsfloat ki_lm2  = 10; 
dsfloat kp_lm2  = 0.2; 
dsfloat i1_lm_ref = 0; 
dsfloat i2_lm_ref = 0; 


dsfloat pos1 = 0; 
dsfloat pos2 = 0; 
dsfloat pos3 = 0; 
dsfloat position3 = 0; 
dsfloat position4 = 0; 

//
// Input signals for ADC   
//

dsfloat input1=0;
dsfloat input2 = 0;
dsfloat input3=0;
dsfloat input4 = 0;
dsfloat input5=0;

//	
// 	some output variables 
// 

dsfloat output1 = 0;
dsfloat output2 = 0;
dsfloat output3 = 0;
dsfloat output4 = 0;
dsfloat output5 = 0; 

dsfloat indexSignal = -1; 
dsfloat subTake = 0; 


//
// LM 1
//

dsfloat kilm1 = 10; 
dsfloat kplm1 = 0.3; 
dsfloat pisition4 = 0; 
dsfloat theta_ref_lm1 = 0.5; 
dsfloat theta_llm1 = 0; 

dsfloat subs1 = 0; 
dsfloat subs2 = 0; 



void isr_timerA(void)
{
	long count,len,count2,len2;
    ts_timestamp_type ts;
    RTLIB_SRT_ISR_BEGIN();                                  /* overload check */
    ts_timestamp_read(&ts); // The absolute time is read and is written to the time stamp structure ts points to.
    host_service(1, &ts);  // To service the data exchange between the real-time hardware and host computer.


	

	//---------------------------------------------------------------------//
	//				Position Control Gearbox Motor 	1					   //
	//---------------------------------------------------------------------//

	position1 = ds3001_read_position(DS3001_2_BASE,1)*100; 
 #if 0 
	theta_m = position1*1.02; 
	error_theta = theta_ref - theta_m; 
	v1_ref = Kp_theta*error_theta; 

	if(v1_ref > 3.1){
		v1_ref = 3.1; 
	}

	if(v1_ref < -3.1){
		v1_ref = -3.1; 
	}
#endif 
	if(position1 >=  position1_prime ){
		dir = 1; 
	}
	else {
		dir = -1; 
	}
		
	position1_prime = position1; 

	//---------------------------------------------------------------------//
	//				Velocity Control Gearbox Motor 	1					   //
	//---------------------------------------------------------------------//

	count = 15; 
	ds4002_f2d_overl ( DS4002_1_BASE, 1, count, &len, &freq1);  // calculates frequency
	v1_motor = dir*freq1/44000;  // reads both rising and falling edges (1/2 -> 22*2kHz) 

#if 0 

	pi1.error = v1_ref - v1_motor; 
	pi_calc(&pi1); 
	error_track = pi1.error; 
	vel_track = pi1.y_out; 


	if(fabs(v1_ref) <= 0.05){ // to nullify the zero reference current. 
		pi1.y_out = 0.0;
	}



	DAC_data[0] = pi1.y_out; 
	#endif 
	
	//---------------------------------------------------------------------//
	//				Position Control Gearbox Motor 2 					   //
	//---------------------------------------------------------------------//

	position2 = ds3001_read_position(DS3001_1_BASE,1)*100; 
#if 0 
	theta_m2 = position2*1.02; 
	error_theta2 = theta_ref2 - theta_m2; 
	v2_ref = Kp_theta2*error_theta2;

	if(v2_ref > 3.1){
		v2_ref = 3.1; 
	}

	if(v2_ref < -3.1){
		v2_ref = -3.1; 
	}
#endif 
	
	if(position2 >=  position2_prime ){
		dir2 = 1; 
	}
	else {
		dir2 = -1; 
	}
		
	position2_prime = position2; 

	//---------------------------------------------------------------------//
	//				Velocity Control Gearbox Motor 	2					   //
	//---------------------------------------------------------------------//

	count2 = 15; 
	ds4002_f2d_overl ( DS4002_1_BASE, 2, count2, &len2, &freq2);  // calculates frequency
	v2_motor = dir2*freq2/44000;  // reads both rising and falling edges (1/2 -> 22*2kHz) 
#if 0 
	pi2.error = v2_ref - v2_motor; 
	pi_calc(&pi2); 
	error_track2 = pi2.error; 
	vel_track2 = pi2.y_out; 

	if(fabs(v2_ref) <= 0.05){ // to nullify the zero reference current. 
		pi2.y_out = 0.0;
	} 

	DAC_data[1] = pi2.y_out;
 #endif 
	//------------------------------------------------------------------------------------------------------// 

	//---------------------------------------------------------------------//
	//				Synchronization of M1 & M2  - velocity				   //
	//---------------------------------------------------------------------//

#if 0 
	pi_s1.error = v2_motor-v1_motor; 
	pi_calc(&pi_s1); 
	i1_ref = 0.5*(pi_s1.y_out+pi_s2.y_out); 

	if(i1_ref > 3.1){
		i1_ref = 3.1; 
	}

	if(i1_ref < -3.1){
		i1_ref = -3.1; 
	}
	DAC_data[0] = i1_ref;

	pi_s2.error = -v1_motor-v2_motor+2*v_ref_s; 
	pi_calc(&pi_s2); 
	i2_ref = 0.5*(pi_s2.y_out-pi_s1.y_out); 

	if(i2_ref > 3.1){
		i2_ref = 3.1; 
	}

	if(i2_ref < -3.1){
		i2_ref = -3.1; 
	}
	DAC_data[1] = i2_ref;
#endif 

	//---------------------------------------------------------------------//
	//				Synchronization of M1 & M2 - position				   //
	//---------------------------------------------------------------------//
	// Kp1,2 = 2, Ki,2 = 25 



	theta_m = position1*1.02; 
	theta_m2 = position2*1.02; 
	pi_s1.error = theta_m2-theta_m; 
	pi_calc(&pi_s1); 
	i1_ref = 0.5*(pi_s1.y_out+pi_s2.y_out); 

	if(i1_ref > 3.1){
		i1_ref = 3.1; 
	}

	if(i1_ref < -3.1){
		i1_ref = -3.1; 
	}

	if(fabs(i1_ref) <= 0.05){ // to nullify the zero reference current. 
		i1_ref = 0.0;
	}


	DAC_data[0] = i1_ref;


	pi_s2.error = -theta_m-theta_m2+2*theta_ref_s; 
	pi_calc(&pi_s2); 
	i2_ref = 0.5*(pi_s2.y_out-pi_s1.y_out); 

	if(i2_ref > 3.1){
		i2_ref = 3.1; 
	}

	if(i2_ref < -3.1){
		i2_ref = -3.1; 
	}

	if(fabs(i2_ref) <= 0.05){ // to nullify the zero reference current. 
		i2_ref = 0.0;
	}

	DAC_data[1] = i2_ref;

 


	//---------------------------------------------------------------------//
	//			LM Synchronization of M1 & M2 - position				   //
	//---------------------------------------------------------------------//

#if 0

	pos_lm1 = ds3001_read_position(DS3001_1_BASE,4)*100; 
	pos_lm2 = ds3001_read_position(DS3001_1_BASE,2)*100; 

	theta_lm1 = 3.27*pos_lm1; 
	theta_lm2 = 2.91*pos_lm2; 
#endif 	


#if 0
	pi_lm1.error = theta_lm2-theta_lm1; 
	pi_calc(&pi_lm1); 
	i1_lm_ref = 0.5*(pi_lm1.y_out+pi_lm2.y_out); 

	if(i1_lm_ref > 3.1){
		i1_lm_ref = 3.1; 
	}

	if(i1_lm_ref < -3.1){
		i1_lm_ref = -3.1; 
	}
#if 0 
	if(fabs(i1_lm_ref) <= 0.05){ // to nullify the zero reference current. 
		i1_lm_ref = 0.0;
	}
#endif 
	DAC_data[2] = i1_lm_ref;



	pi_lm2.error = -theta_lm1-theta_lm2+2*theta_ref_lm; 
	pi_calc(&pi_lm2); 
	i2_lm_ref = 0.5*(pi_lm2.y_out-pi_lm1.y_out); 

	if(i2_lm_ref > 3.1){
		i2_lm_ref = 3.1; 
	}

	if(i2_lm_ref < -3.1){
		i2_lm_ref = -3.1; 
	}
#if 0 
	if(fabs(i2_lm_ref) <= 0.05){ // to nullify the zero reference current. 
		i2_lm_ref = 0.0;
	}
#endif 

	DAC_data[3] = i2_lm_ref;
	#endif 

	//---------------------------------------------------------------------//
	//			LM 1  - position - encoder zeza 						   //
	//---------------------------------------------------------------------//

#if 0 

	theta_llm1 = ds3001_read_position(DS3001_1_BASE,2)*100*2.91; 

	if(theta_llm1 <=0 ){
		ds3001_clear_counter(DS3001_1_BASE,2);
		subs1 = 0; 
		subTake = 0; 
	}
	
	if((theta_llm1 >= 1.46 && subTake <= subs1)){
		subs1 = theta_llm1-1.46; 
		subTake = subs1;
	}

	theta_llm1 = theta_llm1-subs1; 
#endif 

#if 0 

	if(theta_lm1 <=0 ){
		ds3001_clear_counter(DS3001_1_BASE,4);
	}

	if(theta_lm1 >= 1.29){
		subs2 = theta_lm1-1.29; 
	}
	
	theta_lm2 = theta_lm2-subs1; 
	theta_lm1 = theta_lm1-subs2; 

#endif 
#if 0 
	pi_lm_motor1.error = theta_ref_lm1-theta_llm1; 
	pi_calc(&pi_lm_motor1); 
	if(pi_lm_motor1.y_out > 3.1 ){
		pi_lm_motor1.y_out = 3.1; 
	}

	if(pi_lm_motor1.y_out < -3.1 ){
		pi_lm_motor1.y_out = -3.1; 
	}

	if(fabs(pi_lm_motor1.y_out) < 0.01 ){
		pi_lm_motor1.y_out = 0; 
	}
	output2 = pi_lm_motor1.y_out; 
	DAC_data[2] = pi_lm_motor1.y_out; 
#endif 
#if 0 
	DAC_data[2] = output2; 
	indexSignal = ds3001_read_index(DS3001_2_BASE, 2);
#endif 


    ds2103_out(DS2103_1_BASE,1,DAC_data);
    //start AD conversion
    ds2003_start(DS2003_1_BASE);
    ds2003_in(DS2003_1_BASE, ADC_data);
	
	input1 = ADC_data[0]*10.0;
	input2 = ADC_data[1]*10.0;
	input3 = ADC_data[2]*10.0;
	input4 = ADC_data[3]*10.0;
	input5 = ADC_data[4]*10.0;
	
	
	pi1.K_i = ki; 
	pi1.K_p = kp; 

	pi2.K_i = ki2; 
	pi2.K_p = kp2;	

	pi_s1.K_i = ki_s1; 
	pi_s1.K_p = kp_s1; 

	pi_s2.K_i = ki_s2; 
	pi_s2.K_p = kp_s2; 

	pi_lm2.K_i = ki_lm2; 
	pi_lm2.K_p = kp_lm2; 

	pi_lm1.K_i = ki_lm1; 
	pi_lm1.K_p = kp_lm1; 


	pi_lm_motor1.K_i = kilm1; 
	pi_lm_motor1.K_p = kplm1; 


	RTLIB_SRT_ISR_END();
}




void main()
{
    
    init();                            /* Main DS1005 initialization*/            
    //host_service(0,0)                  /* ??? */
    

	//---------------------------------------------------------------------//
	//						Motor 1 Defaults  							   //
	//---------------------------------------------------------------------//

	pi1.error = 0; 
	pi1.error_prev = 0; 
	pi1.y_prev = 0; 
	pi1.y_out = 0; 
	pi1.K_i = 10; 
	pi1.K_p = 0.1; 
	pi1.T = 1e-5; 
	pi1.upperLimit = 3; 
	pi1.lowerLimit = -3; 

	//---------------------------------------------------------------------//
	//						Motor 2 Defaults  							   //
	//---------------------------------------------------------------------//

	pi2.error = 0; 
	pi2.error_prev = 0; 
	pi2.y_prev = 0; 
	pi2.y_out = 0; 
	pi2.K_i = 10; 
	pi2.K_p = 0.1; 
	pi2.T = 1e-5; 
	pi2.upperLimit = 3; 
	pi2.lowerLimit = -3; 

	//---------------------------------------------------------------------//
	//						Sync Defaults 	 							   //
	//---------------------------------------------------------------------//

	pi_s1.error = 0; 
	pi_s1.error_prev = 0; 
	pi_s1.y_prev = 0; 
	pi_s1.y_out = 0; 
	pi_s1.K_i = -10; 
	pi_s1.K_p = -0.1; 
	pi_s1.T = 1e-5; 
	pi_s1.upperLimit = 3; 
	pi_s1.lowerLimit = -3; 


	pi_s2.error = 0; 
	pi_s2.error_prev = 0; 
	pi_s2.y_prev = 0; 
	pi_s2.y_out = 0; 
	pi_s2.K_i = -10; 
	pi_s2.K_p = -0.1; 
	pi_s2.T = 1e-5; 
	pi_s2.upperLimit = 3; 
	pi_s2.lowerLimit = -3; 


	//---------------------------------------------------------------------//
	//						LM Sync Defaults 	 						   //
	//---------------------------------------------------------------------//

	pi_lm1.error = 0; 
	pi_lm1.error_prev = 0; 
	pi_lm1.y_prev = 0; 
	pi_lm1.y_out = 0; 
	pi_lm1.K_i = -10; 
	pi_lm1.K_p = -0.1; 
	pi_lm1.T = 1e-5; 
	pi_lm1.upperLimit = 1; 
	pi_lm1.lowerLimit = -1; 


	pi_lm2.error = 0; 
	pi_lm2.error_prev = 0; 
	pi_lm2.y_prev = 0; 
	pi_lm2.y_out = 0; 
	pi_lm2.K_i = -10; 
	pi_lm2.K_p = -0.1; 
	pi_lm2.T = 1e-5; 
	pi_lm2.upperLimit = 1; 
	pi_lm2.lowerLimit = -1; 

	

	//---------------------------------------------------------------------//
	//						LM 1 Defaults 	 							   //
	//---------------------------------------------------------------------//

	pi_lm_motor1.error = 0; 
	pi_lm_motor1.error_prev = 0; 
	pi_lm_motor1.y_prev = 0; 
	pi_lm_motor1.y_out = 0; 
	pi_lm_motor1.K_i = 10; 
	pi_lm_motor1.K_p = 0.1; 
	pi_lm_motor1.T = 1e-5; 
	pi_lm_motor1.upperLimit = 3; 
	pi_lm_motor1.lowerLimit = -3; 


	/*************DS2103 (DAC Board) initializations*******************************/
	ds2103_init(DS2103_1_BASE);   										/* Initialize the DAC board*/                                  
	ds2103_set_errmode(DS2103_1_BASE, DS2103_CH_ALL, DS2103_KEEP);      /* If I/O error occurs keep all outputs unchanged*/
	ds2103_set_outmode(DS2103_1_BASE, DS2103_CH_ALL, DS2103_TRANS);     /* Output mode of the D/A converters*/
	ds2103_set_range(DS2103_1_BASE, DS2103_CH_ALL,DS2103_RNG10);        /* Output voltage range of the D/A converter(+/-10V)*///DS2103_RNG10
	ds2103_init_scantbl(DS2103_1_BASE, 1, DACCOUNT, dacchannels);       /* Specify DAC channels to be used*/                                                              
	/******************************************************************************/
	
	/*************DS3001 (ENCODER Board) initializations**************************/
	ds3001_init(DS3001_1_BASE);									        /* Initialize the encoder board 1*/
    ds3001_init(DS3001_2_BASE);                                         /* Initialize the encoder board 2*/
    ds3001_set_line_type(DS3001_1_BASE, DS3001_CH_ALL, DS3001_SINGLE);  /* Single ended input signal */
    ds3001_set_line_type(DS3001_2_BASE, DS3001_CH_ALL, DS3001_SINGLE);  /* Single ended input signal */
    ds3001_clear_lerr(DS3001_1_BASE);                                   /* Clear the latched I/O error flag  */
    ds3001_clear_lerr(DS3001_2_BASE);                                   /* Clear the latched I/O error flag  */
	/*****************************************************************************/


    /*************DS2003 (ADC Board) initializations*******************************/
    ds2003_board_init(DS2003_1_BASE);                                   /* Initialize the AD board*/
    ds2003_set_range(DS2003_1_BASE, DS2103_CH_ALL, DS2003_RNG10);       /* Input voltage range of the A/D converter(+/-5V)*/
    ds2003_set_wordlen(DS2003_1_BASE, DS2003_CH_ALL, DS2003_LEN16);     /* Wordlength, 16bit*/
    ds2003_init_scantbl(DS2003_1_BASE, ADCCOUNT, adcchannels);          /* Specify ADC channels to be used*/
    /******************************************************************************/
	
	/*************DS4002 (IO Board) initializations*******************************/
	
	
	ds4002_init(DS4002_1_BASE);
    ds4002_pwm2d_init (DS4002_1_BASE, 1, 0, 0.0);
    ds4002_pwm2d_init (DS4002_1_BASE, 2, 0, 0.0);

	#if 0 
		ds4002_pwm_init ( DS4002_1_BASE, 1, 1/freq1, duty1);
		ds4002_pwm_init ( DS4002_1_BASE, 2, 1/freq2, duty2);
		ds4002_pwm_init ( DS4002_1_BASE, 3, 1/freq3, duty3);
		ds4002_pwm_init ( DS4002_1_BASE, 4, 1/freq4, duty4);
		ds4002_pwm_init ( DS4002_1_BASE, 5, 1/freq5, duty5);
		ds4002_pwm_init ( DS4002_1_BASE, 6, 1/freq6, duty6);
		ds4002_pwm_init ( DS4002_1_BASE, 7, 1/freq7, duty7);
		ds4002_pwm_init ( DS4002_1_BASE, 8, 1/freq8, duty8);
	#endif 
	 /******************************************************************************/


    msg_info_set(MSG_SM_RTLIB, 0, "System started.");                   /* To generate an information message.*/ 
	RTLIB_SRT_START(DT, isr_timerA);                                    /* Start sample rate timer */


	while (1)
	{
		RTLIB_BACKGROUND_SERVICE();
	}
}



