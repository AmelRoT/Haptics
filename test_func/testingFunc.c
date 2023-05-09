#include <stdio.h>      /* STandard Digital I/O*/ 
#include <brtenv.h>     /* Basic Real Time ENViroment*/
#include <dstypes.h>	/* Standard Types Aupported by Dspace*/
#include <hostsvc.h>	/* Host Service*/
#include <ds3001.h>     /* Incremental Encoder Interface Board*/
#include <ds2103.h>     /* Multi-Channel D/A Converter Board*/  
#include <ds4002.h>     /* DS4002 Digital I/O Board*/  
#include <ds2003.h>     /* Multi-Channel A/D Converter Board*/
#include <math.h>       /* Math library*/
#include "readPosVel.c"
//#include "dob.c"
#include "sign_fun.c"
#include "friction_est.c"
#include "dob_friction.c"
#include "dob.c"
//#define     DT      0.75e-4          /* 75 us simulation step size */
// #define     DT      0.75e-4         /* 75 us simulation step size */
#define     DT      1e-4 
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

struct readPosVel reading; 
//struct dob dob1 = defaultDOB; 
struct dob_f1 dob2 = defaultDOB_friction; 
struct dob dob1 = defaultDOB; 
struct dob dob3 = defaultDOB; 




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



dsfloat C22 = 100; 
dsfloat K_p_sigma2 = 0.2; 
dsfloat sat_sigma = 10; 
dsfloat f_env_read;
dsfloat sigma_mot_read;
dsfloat i_sine_friction = 0; 
dsfloat i_sine_friction2 = 0.5; 
dsfloat changeG = 100; 
dsfloat var1 = 1; 

dsfloat value1 = 10; 
dsfloat value2 = 20; 
dsfloat value3 = -1; 
dsfloat sign_func1_test = 0; 
dsfloat disT; 
dsfloat disT2; 
dsfloat v1_prava; 
dsfloat G1;

dsfloat w1 = 0; 
dsfloat t2 = 0; 
dsfloat theta_2 =0; 
dsfloat T_dis3; 
//#define macroFunction(x,y) x*y 

#define macroFunction(x,y,c) 	\
	if(c == 1){  \
		x = 100; \
		y = 200; \
	} 			

#define macro2(x) \
	 x = 300; 



#define sign1(x,y1) 	 \
	if(x >= 0) { 	 	 \
		y1 = 10;	     \
	}				 	 \
	else{			 	 \
		y1 = 20;		 \
	}              


#if 0 
#define anotherMacro(x,y,y1) \
	x = 100*y*y1; \
	y = x*2; 
#endif 
dsfloat test_output = 0; 



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

	
	
//	test_output = 0.1*sin(2*pi*t1*0.1); 
	

	
	reading.i_dir1= test_output; 

	measurePositonVel(&reading);

	theta_m =  reading.position1*1.02; 
	theta_m2 = reading.position2*1.02; 


#if 0 
	w1 = (theta_m - theta_2)/(t1-t2); 

	t2 = t1; 
	theta_2 = theta_m; 
#endif 



	v1_motor = reading.vel1; 
	v2_motor = reading.vel2;



	if(fabs(v1_motor)<=0.02){
		v1_motor = 0; 
	}


	v1_prava = v1_motor*1; 
	

	dob1.w_n = v1_prava; 
	dob1.i_n = test_output; 
	calc_DOB(&dob1);
	disT2 = dob1.T_n; 

#if 0 
 
	dob2.w_n = v1_motor; 
	dob2.i_n = test_output; 
	calc_DOB_friction(&dob2); 

	disT = dob2.T_n*43; 
	G1 = dob2.G; 

#endif 

	dob3.w_n = v1_motor; 
	dob3.i_n = test_output; 

	calc_DOB(&dob3);

	T_dis3 = dob3.T_n-calcFriction(v1_motor); 
		

	// disT = dob1.T_n; 
	//disT2 = dob1.T_n; 

	macro2(value1); 
	sign1(value3,value2); 




#if 0
	//  sign1(value2); 
 	//	sing11(value1);
	//  value3 = anotherMacro(value1,value2,value3);

	//macroFunction(value1,value2,value3); 
	//macroFunction(value1,value2,1); 
	//macroFunction(value1,value2,value3); 
 	// DAC_data[1] = syncGearBox.i2_ref;
#endif 
	DAC_data[0] = test_output;



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



