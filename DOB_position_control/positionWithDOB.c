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
//#define     DT      0.75e-4          /* 75 us simulation step size */
// #define     DT      0.75e-4         /* 75 us simulation step size */
#define     DT      0.4e-4 

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
// DOB structs 
// 

struct dob T_dis; 
struct velocityTrack vel; 
struct readPosVel reading = defaultRead;
//
// For the sync 
// 
struct dob T_dis1; 
struct dob T_dis2; 
struct syncWithDobb synC; 
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

dsfloat i_friction = 0; 
dsfloat T_measured = 0; 



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
	theta_m2 = reading.position2*1.02; 

	v1_motor = reading.vel1; 
	v2_motor = reading.vel2; 



	i_friction = 0.04*sin(2*pi*t1*0.3); 

	vel.v_mes = v1_motor; 
	vel.i_ref = i_friction; 

	fricitonPositionEst(&T_dis,&vel); 

	T_measured = T_dis.T_n; 
	
	DAC_data[0] = i_friction;
	 



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

	theta_m = position1*1.02; 
	vel.theta_mes = theta_m; 
	vel.theta_ref = theta_ref; 
	positionTrack(&T_dis,&vel); 
	DAC_data[0] = vel.i_ref; 

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



    msg_info_set(MSG_SM_RTLIB, 0, "System started.");                   /* To generate an information message.*/ 
	RTLIB_SRT_START(DT, isr_timerA);                                    /* Start sample rate timer */


	while (1)
	{
		RTLIB_BACKGROUND_SERVICE();
	}
}



