#include <stdio.h>      /* STandard Digital I/O*/ 
#include <brtenv.h>     /* Basic Real Time ENViroment*/
#include <dstypes.h>	/* Standard Types Aupported by Dspace*/
#include <hostsvc.h>	/* Host Service*/
#include <ds3001.h>     /* Incremental Encoder Interface Board*/
#include <ds2103.h>     /* Multi-Channel D/A Converter Board*/  
#include <ds4002.h>     /* DS4002 Digital I/O Board*/  
#include <ds2003.h>     /* Multi-Channel A/D Converter Board*/
#include <math.h>       /* Math library*/
#include "dob.c"
#include "velocityWithDOB.c"
#include "syncWithDOB.c" 
#include "readPosVel.c"
#include "sigma_dob.c"
#include "sigma_motion_1DOB.c"
#include "sigma_motion_2DOB.c"
#include "sigma_motion_3DOB_force.c"
#include "friciton_comp.h"
#include "sign_fun.c"
#include "lowPassDer.c"



//#define     DT      0.75e-4          /* 75 us simulation step size */
// #define     DT      0.75e-4         /* 75 us simulation step size */
#define     DT      1.5e-4
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

struct dob T_dis; 
struct velocityTrack vel; 
struct readPosVel reading = defaultRead;

struct sigma_motion_1DOB sigma_values = sigma_motionDefaults; 
struct sigma_dob dob_s = defaultDOBSigma; 

struct sigma_motion_2DOB sigma_values2 = sigma_motionDefaults; 
struct sigma_dob dob_s2 = defaultDOBSigma; 
struct dob dob11 = defaultDOB; 


struct sigma_motion_3DOB_force sigma_force = sigma_motion_3DOB_force_defaults; 
struct sigma_dob dob_s3 = defaultDOBSigma; 
struct dob dob3 = defaultDOB; 


struct sigma_motion_3DOB_force sigma_force2 = sigma_motion_3DOB_force_defaults; 
struct sigma_dob dob_s3_motor2 = defaultDOBSigma; 
struct dob dob3_motor2 = defaultDOB; 



//
// For the sync 
// 
struct dob T_dis1; 
struct dob T_dis2; 
struct syncWithDobb synC; 
struct derivative deriv = defaultValuesDer; 
struct derivative deriv2 = defaultValuesDer; 

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

dsfloat K_pp = 10.5; 
dsfloat sine_wave = 0; 
dsfloat t1 = 0; 

dsfloat K_p1 = 10;
dsfloat K_p2 = 10; 

dsfloat error1_plot = 0; 
dsfloat error2_plot = 0; 

dsfloat T_dis_read1 = 0; 
dsfloat T_dis_read2 = 0; 
dsfloat C11 = 100; 
dsfloat K_p_sigma1 = 0.2;  



dsfloat C22 = 150; 
dsfloat K_p_sigma2 = -0.626; 
dsfloat K_p_sigma22 = -0.5; 

dsfloat sat_sigma = 0.1; 
dsfloat sat_sigma2 = 0.1; 

dsfloat f_env_read;
dsfloat sigma_mot_read; 
dsfloat f_env_read2; 
dsfloat sigma_mot_read2;

dsfloat T_dis_read_friction = 0; 
dsfloat i_ref_friction = 0; 
dsfloat test_sign = 0; 
dsfloat valueTest = 10; 
dsfloat readT_dis = 0; 
dsfloat i_read = 0; 
dsfloat dummySignal = 0; 



void isr_timerA(void)
{

//	long count,len,count2,len2;
    ts_timestamp_type ts;
    RTLIB_SRT_ISR_BEGIN();                                  /* overload check */

	
    ts_timestamp_read(&ts); // The absolute time is read and is written to the time stamp structure ts points to.
    host_service(1, &ts);  // To service the data exchange between the real-time hardware and host computer.


	//---------------------------------------------------------------------//
	//				Positions and velcites from Encoder					   //
	//---------------------------------------------------------------------//

	t1 = t1+DT; 

	measurePositonVel(&reading); 
	

	theta_m =  reading.position1*1.02; 
	theta_m2 = reading.position2*20; 
	v1_motor = reading.vel1/20; 
//	v2_motor = reading.vel2*20;
	v2_motor = reading.vel2/20;
	//v1_motor = reading.vel1; 
	// v1_motor = v2_motor; 
#if 0 
	test_sign = sign(valueTest); 
	 #endif 


	//---------------------------------------------------------------------//
	//						Sigma Motion DOB 							   //
	//---------------------------------------------------------------------//

	// Big motor -> K_p sigma = -0.1 , C11 = 50 - 70
	
	// theta_ref = 0.15*sin(2*pi*0.1*t1); 
#if 0 

	sigma_values.theta1_mes = theta_m; 
	sigma_values.theta1_ref = theta_ref; 

	sigma_values.v1_mes = v1_motor; 
	sigma_values.v1_ref = 0; 
	sigma_values.C1 = C11; 
	sigma_values.K_p_sigma = K_p_sigma1; 

	sigmaCalculate(&sigma_values,&dob_s);
	readT_dis = dob_s.T_n; 

	DAC_data[2] = sigma_values.i1_ref; 
#endif 
	// For LM -> C11 = 250, K_p_sigma = -0.2, theta_ref = +-0.25

	//---------------------------------------------------------------------//
	//						Sigma Motion DOB2 							   //
	//---------------------------------------------------------------------//

 #if 0 
	sigma_values2.theta1_mes = theta_m; 
	sigma_values2.theta1_ref = theta_ref; 

	sigma_values2.v1_mes = v1_motor; 
	sigma_values2.v1_ref = 0; 
	sigma_values2.C1 = C22; 
	sigma_values2.K_p_sigma = K_p_sigma2; 

	sigmaCalculate2DOB(&sigma_values2,&dob_s2,&dob11);

	DAC_data[2] = sigma_values2.i1_ref; 
#endif 

	// For LM -> C22 = 50, K_p_sigma = -0.5, theta_ref = +-0.25
	
	
	// For LM -> C22 = 150, K_p_sigma = -0.626, theta_ref = +-0.25

	//---------------------------------------------------------------------//
	//						Sigma Motion DOB3 - force 					   //
	//---------------------------------------------------------------------//
 
	// i_ref_friction = 0.03*sin(2*pi*0.3*t1); 
#if 0
	theta_ref = 0.15*sin(2*pi*0.1*t1); 
//	sat_sigma = 0.2+0.03*sin(2*pi*0.2*t1); 
//	sat_sigma = 0.2; 

	sigma_force.theta1_mes = theta_m; 
	sigma_force.theta1_ref = theta_ref; 
	sigma_force.v1_mes = v1_motor; 
	deriv.x_n = v1_motor; 
	calcDerivative(&deriv); 
	sigma_force.v1_ref = deriv.y_n; 
	sigma_force.C1 = C22; 
	sigma_force.K_p_sigma = K_p_sigma2; 
	sigma_force.saturation_sigma = sat_sigma; 
	sigmaCalculate3DOB(&sigma_force,&dob_s3,&dob3);

	f_env_read = sigma_force.f_env; 
	sigma_mot_read = sigma_force.sigma_mot; 
	DAC_data[2] = sigma_force.i1_ref; 
	i_read = sigma_force.i1_ref;

#endif 
	//
	// DC Big motor 
	// 
	// For DC-Big -> C22 = 150, K_p_sigma22 = -0.01

	theta_ref = 0.1*sin(2*pi*0.1*t1); 
	sigma_force2.theta1_mes = theta_m2; 
	sigma_force2.theta1_ref = theta_ref; 
	sigma_force2.v1_mes = v2_motor; 
	deriv.x_n = 0; 
	calcDerivative(&deriv2);
	sigma_force2.v1_ref = deriv2.y_n; 
	sigma_force2.C1 = C22; 
	sigma_force2.K_p_sigma = K_p_sigma22; 
	sigma_force2.saturation_sigma = sat_sigma2; 
	sigmaCalculate3DOB(&sigma_force2,&dob_s3_motor2,&dob3_motor2);
	f_env_read2 = sigma_force2.f_env; 
	sigma_mot_read = sigma_force2.sigma_mot; 
	DAC_data[3] = sigma_force2.i1_ref; 
	i_read = sigma_force2.i1_ref;

	
#if 0 
	i_ref_friction = 0.03*sin(2*pi*0.3*t1); 
	T_dis_read_friction = dob3.T_n;	
	DAC_data[0] = i_ref_friction; 
	DAC_data[1] = i_ref_friction; 
#endif 



	//---------------------------------------------------------------------//
	//				Velocity with DOB 									   //
	//---------------------------------------------------------------------//
#if 0 
	vel.v_mes = v1_motor; 
	vel.v_ref = v1_ref; 
	velocityTrack(&T_dis,&vel); 
    DAC_data[0] = vel.i_ref;
#endif 




	//---------------------------------------------------------------------//
	//				position with DOB 									   //
	//---------------------------------------------------------------------//

	
#if 0
	//theta_m = position1*1.02; 
	vel.theta_mes = theta_m; 
	vel.theta_ref = theta_ref; 
	positionTrack(&T_dis,&vel); 
	DAC_data[0] = vel.i_ref; 
	i_read = vel.i_ref; 
#endif 
	// 
	// sine ref 
	// 

#if 0 
	sine_wave = sin(2*pi*2*t1); 
	theta_m = position1*1.02; 
	vel.theta_mes = theta_m; 
	vel.theta_ref = sine_wave; 
	positionTrack(&T_dis,&vel); 
	DAC_data[0] = vel.i_ref; 

#endif 




	//theta_ref = sin(2*pi*0.3*t1); 
	//theta_ref = 3; 

#if 0 
	synC.theta_mes1 = theta_m; 
	synC.theta_mes2 = theta_m2; 
	synC.theta_ref =  theta_ref; 

	syncDOBposition(&T_dis1, &T_dis2, &synC); 
	

	error1_plot = synC.error1; 
	error2_plot = synC.error2; 
	

	T_dis_read1 = T_dis1.T_n; 
	T_dis_read2 = T_dis2.T_n; 

	DAC_data[0] = synC.i_ref1; 
	DAC_data[1] = synC.i_ref2; 
#endif 

 	// DAC_data[1] = syncGearBox.i2_ref;
	
	vel.K_p   = K_pp; 
	synC.K_p1 = K_p1; 
	synC.K_p2 = K_p2; 


	//DAC_data[3] = dummySignal; 

    ds2103_out(DS2103_1_BASE,1,DAC_data);
    //start AD conversion
    ds2003_start(DS2003_1_BASE);
    ds2003_in(DS2003_1_BASE, ADC_data);

	//---------------------------------------------------------------------//
	//				Real Time adjustment of Ki and Kp parameters		   //
	//---------------------------------------------------------------------//


	RTLIB_SRT_ISR_END();
}




void main()
{

    
    init();                            /* Main DS1005 initialization*/        

    //host_service(0,0)                  /* ??? */
    
	//---------------------------------------------------------------------//
	//						Sync Defaults 	 							   //
	//---------------------------------------------------------------------//


	T_dis. T_n = 0 ; 
	T_dis. T_n_1 = 0; 
	T_dis. i_n= 0; 
	T_dis. i_n_1= 0;
	T_dis. w_n= 0; 
	T_dis. w_n_1= 0;    
	T_dis. C1 = 0; 
	T_dis. g = 100;  
	T_dis. T = 1e-5; 
	T_dis. J = 0.001; 
	T_dis. Kt = 0.8; 

    vel.error = 0; 
    vel.i_con = 0; 
    vel.K_p = 10; 
    vel.v_mes = 0; 
    vel.v_ref = 0; 
    vel.i_ref = 0; 
	vel.theta_mes = 0; 
	vel.theta_ref = 0; 

	//--------------------------------------------------------------//


	T_dis1. T_n = 0 ; 
	T_dis1. T_n_1 = 0; 
	T_dis1. i_n= 0; 
	T_dis1. i_n_1= 0;
	T_dis1. w_n= 0; 
	T_dis1. w_n_1= 0;    
	T_dis1. C1 = 0; 
	T_dis1. g = 100;  
	T_dis1. T = 1e-5; 
	T_dis1. J = 0.001; 
	T_dis1. Kt = 0.8; 

	T_dis2. T_n = 0 ; 
	T_dis2. T_n_1 = 0; 
	T_dis2. i_n= 0; 
	T_dis2. i_n_1= 0;
	T_dis2. w_n= 0; 
	T_dis2. w_n_1= 0;    
	T_dis2. C1 = 0; 
	T_dis2. g = 100;  
	T_dis2. T = 1e-5; 
	T_dis2. J = 0.001; 
	T_dis2. Kt = 0.8; 



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
    ds4002_pwm2d_init (DS4002_1_BASE, 4, 0, 0.0);


    msg_info_set(MSG_SM_RTLIB, 0, "System started.");                   /* To generate an information message.*/ 
	RTLIB_SRT_START(DT, isr_timerA);                                    /* Start sample rate timer */


	while (1)
	{
		RTLIB_BACKGROUND_SERVICE();
	}
}



