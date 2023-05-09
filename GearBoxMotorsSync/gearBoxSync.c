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
#include "sync.c"

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

struct Pi pi_s1; 
struct Pi pi_s2; 
struct sync syncGearBox; 


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



void isr_timerA(void)
{

	long count,len,count2,len2;
    ts_timestamp_type ts;
    RTLIB_SRT_ISR_BEGIN();                                  /* overload check */
    ts_timestamp_read(&ts); // The absolute time is read and is written to the time stamp structure ts points to.
    host_service(1, &ts);  // To service the data exchange between the real-time hardware and host computer.


	

	//---------------------------------------------------------------------//
	//				Positions and velcites from Encoder					   //
	//---------------------------------------------------------------------//

	position1 = ds3001_read_position(DS3001_1_BASE,1)*100; 

	if(position1 >=  position1_prime ){

		dir = 1; 
	}
	else {

		dir = -1; 
	}
		
	position1_prime = position1; 
	count = 15; 
	ds4002_f2d_overl (DS4002_1_BASE, 1, count, &len, &freq1);  // calculates frequency
	v1_motor = dir*freq1/44000;  // reads both rising and falling edges (1/2 -> 22*2kHz) 

	position2 = ds3001_read_position(DS3001_1_BASE,4)*100; 
	
	if(position2 >=  position2_prime ){
		dir2 = 1; 
	}
	else {
		dir2 = -1; 
	}
		
	position2_prime = position2; 
	count2 = 15; 
	ds4002_f2d_overl ( DS4002_1_BASE, 2, count2, &len2, &freq2);  // calculates frequency
	v2_motor = dir2*freq2/44000;  // reads both rising and falling edges (1/2 -> 22*2kHz) 


	//---------------------------------------------------------------------//
	//				Synchronization of M1 & M2 - position				   //
	//---------------------------------------------------------------------//


	syncGearBox.theta_m1 = position1; 
	syncGearBox.theta_m2 = position2; 
	syncGearBox.theta_ref = theta_ref_s; 

	calcSyncPosition(&pi_s1, &pi_s2, &syncGearBox); 

	DAC_data[0] = syncGearBox.i1_ref;
	DAC_data[1] = syncGearBox.i2_ref;




    ds2103_out(DS2103_1_BASE,1,DAC_data);
    //start AD conversion
    ds2003_start(DS2003_1_BASE);
    ds2003_in(DS2003_1_BASE, ADC_data);

	
	//---------------------------------------------------------------------//
	//				Real Time adjustment of Ki and Kp parameters		   //
	//---------------------------------------------------------------------//

	pi_s1.K_i = ki_s1;  
	pi_s1.K_p = kp_s1; 

	pi_s2.K_i = ki_s2; 
	pi_s2.K_p = kp_s2; 



	RTLIB_SRT_ISR_END();
}




void main()
{
    
    init();                            /* Main DS1005 initialization*/            
    //host_service(0,0)                  /* ??? */
    
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


	syncGearBox.theta_m1 = 0; 
	syncGearBox.theta_m2 = 0; 
	syncGearBox.theta_ref = 2; 
	syncGearBox.i1_ref = 0; 
	syncGearBox.i2_ref = 0; 



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



