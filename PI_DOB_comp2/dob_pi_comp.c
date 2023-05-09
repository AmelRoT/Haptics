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
#include "dob_theta.c"
#include "pid.c"
#include "lowPassDer.c"
#include "dob_vel.c"
#include "pi.c"


//#define     DT      0.75e-4          /* 75 us simulation step size */
// #define     DT      0.75e-4         /* 75 us simulation step size */
//#define     DT      0.2e-2 
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
//dsfloat GIO_DATA[GPIOACCOUNT] = {0,0,0,0,0,0,0,0};



//------------------------------------------------------------------------------//
//							Global Variables 									//
//------------------------------------------------------------------------------//

//
// DOB structs 
// 


struct Pi e1 = defaultSyncVel; 
struct Pi e2= defaultSyncVel; 

struct Pi e11_pi = defaultSyncVel; 
struct Pi e22_pi = defaultSyncVel; 


struct PID e11 = defaultValuePID; 
struct PID e22 = defaultValuePID; 

struct dob_vel e111 = defaultValueDOBvel; 
struct dob_vel e222 = defaultValueDOBvel; 

struct dob_theta e1111 = defaultValueDOBtheta; 
struct dob_theta e2222 = defaultValueDOBtheta; 

struct derivative der1 = defaultValuesDer; 
struct derivative der2 = defaultValuesDer; 





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
//dsfloat Kp_theta = 25; 
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
dsfloat theta_ref = 1; 
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
dsfloat theta_ref2 = 1; 
dsfloat theta_m2 = 0; 


dsfloat t1 = 0; 
dsfloat v_ref  = 1.5; 
dsfloat i1_ref = 0; 
dsfloat i2_ref = 0; 
dsfloat J = 0.0001; 
dsfloat Kt = 0.8; 

dsfloat Ki1 = -40; 
dsfloat Ki2 = -40; 

dsfloat Kp1 = -0.5; 
dsfloat Kp2 = -0.5; 

dsfloat Kd1 = 0; 
dsfloat Kd2 = 0; 

dsfloat dob_vel_e1 = 0; 
dsfloat dob_vel_e2 = 0; 
dsfloat Kp_dobVel = -0.1; 
dsfloat Kp_dobTheta = -10; 

dsfloat dob_theta_e1 = 0; 
dsfloat dob_theta_e2 = 0; 
dsfloat Kp_theta = -10; 
dsfloat Kd_theta = -1; 

dsfloat xi_minus; 
dsfloat xi_plus; 


dsfloat T_dis_vel1; 
dsfloat T_dis_vel2; 

dsfloat error_motor1; 
dsfloat error_motor2; 

dsfloat value111; 
dsfloat value222; 
dsfloat  var1112; 
dsfloat delta_X; 


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

	
//	measurePositonVel(&reading); 
	measurePositonVel(&reading); 
	theta_m =  reading.position1*1.02; 
	theta_m2 = reading.position2*1.02; 

	v1_motor = reading.vel1; 
	v2_motor = reading.vel2; 

	delta_X  = ds3001_read_delta_position(DS3001_1_BASE,4); 
	DAC_data[2] = var1112; 


	// LM Motor 1 -> bnc 4. --> DAC 3
	// LM Motor 2 -> bnc 3. --> DAC 2 
	//-----------------------------------------------------------------------------------//
	// 								PI Velocity Sync 									 // 
	//-----------------------------------------------------------------------------------//
	// Kp = -1, Ki = -200  reference constant signal 
	// Kp = -1, Ki = -250  reference constant signal 
 

  //  v_ref = 1.0*sin(2*pi*0.1*t1); 

	e1.error = v1_motor-v2_motor; 
	pi_calc(&e1); 

	e2.error = v1_motor+v2_motor-2*v_ref; 
	pi_calc(&e2); 

	//i1_ref = ((e1.y_out+e2.y_out)/2)*(J/Kt); 

	i1_ref = ((e1.y_out+e2.y_out)/2); 

	if(i1_ref > 3.1){
		i1_ref  = 3.1; 
	}

	if(i1_ref < -3.1){
		
		i1_ref = -3.1; 
	}

	if(fabs(i1_ref) <= 0.05 ){  // to nullify the zero reference current. 
		i1_ref   = 0.0;
	}

	
//	i2_ref = (-e1.y_out+e2.y_out)/2*(J/Kt); 
	i2_ref = (-e1.y_out+e2.y_out)/2; 

	if(i2_ref > 3.1){
		i2_ref  = 3.1; 
	}

	if(i2_ref < -3.1){
		
		i2_ref = -3.1; 
	}

	if(fabs(i2_ref) <= 0.05 ){  // to nullify the zero reference current. 
		i2_ref   = 0.0;
	}	
	
	DAC_data[0] = i1_ref;
	DAC_data[1] = i2_ref;

	error_motor1 = e1.error; 
	error_motor2 = e2.error; 

	e1.Ki = Ki1; 
	e1.Kp = Kp1; 
	e2.Ki = Ki2; 
	e2.Kp = Kp2; 


	//-----------------------------------------------------------------------------------//
	// 								PID theta Sync 										 // 
	//-----------------------------------------------------------------------------------//

	// Ki = -65, Kp = -1

	// Kp = -5
	// Ki = -60
	// Kd = -0.2 -> -0.02 -> -0.0002 

  //  theta_ref = 1.0*sin(2*pi*0.1*t1); 

#if 0 
	e11.error_n = theta_m-theta_m2; 
	pid_calc(&e11); 


	e22.error_n = theta_m+theta_m2-2*theta_ref; 
	pid_calc(&e22); 

	i1_ref = (e11.y_n+e22.y_n)/2; 

	if(i1_ref > 3.1){
		i1_ref  = 3.1; 
	}

	if(i1_ref < -3.1){
		
		i1_ref = -3.1; 
	}
#if 0 
	if(fabs(i1_ref) <= 0.05 ){  // to nullify the zero reference current. 
		i1_ref   = 0.0;
	}
#endif 
	
	i2_ref = (-e11.y_n+e22.y_n)/2; 

	if(i2_ref > 3.1){
		i2_ref  = 3.1; 
	}

	if(i2_ref < -3.1){
		
		i2_ref = -3.1; 
	}
#if 0 
	if(fabs(i2_ref) <= 0.05 ){  // to nullify the zero reference current. 
		i2_ref   = 0.0;
	}
#endif 

#if 0 
	DAC_data[0] = i1_ref;
	DAC_data[1] = i2_ref;
#endif

	DAC_data[3] = i1_ref;
	DAC_data[2] = i2_ref;

	error_motor1 = e11.error_n; 
	error_motor2 = e22.error_n;


	e11.Ki = Ki1; 
	e11.Kp = Kp1; 
	e11.Kd = Kd1; 

	e22.Ki = Ki2; 
	e22.Kp = Kp2; 
	e22.Kd = Kd2; 

#endif 


	// -1000 = Ki , Kp = -5 
 	// LM -> motor Ki = -3000, Kp = -10 for both M1 and M2  
 	// theta_ref = 1.0*sin(2*pi*0.1*t1); 
 #if 0 
	e11_pi.error = theta_m-theta_m2; 
	pi_calc(&e11_pi); 


	e22_pi.error = theta_m+theta_m2-2*theta_ref; 
	pi_calc(&e22_pi); 

	i1_ref = (e11_pi.y_out+e22_pi.y_out)/2; 

	if(i1_ref > 3.1){
		i1_ref  = 3.1; 
	}

	if(i1_ref < -3.1){
		
		i1_ref = -3.1; 
	}

#if 0 
	if(fabs(i1_ref) <= 0.05 ){  // to nullify the zero reference current. 
		i1_ref   = 0.0;
	}
#endif 

	i2_ref = (-e11_pi.y_out+e22_pi.y_out)/2; 

	if(i2_ref > 3.1){
		i2_ref  = 3.1; 
	}

	if(i2_ref < -3.1){
		
		i2_ref = -3.1; 
	}
#if 0 
	if(fabs(i2_ref) <= 0.05 ){  // to nullify the zero reference current. 
		i2_ref   = 0.0;
	}
#endif 
#if 0 
	DAC_data[0] = i1_ref;
	DAC_data[1] = i2_ref;
#endif 

	DAC_data[3] = i1_ref;
	DAC_data[2] = i2_ref;

	error_motor1 = e11_pi.error; 
	error_motor2 = e22_pi.error;


	e11_pi.Ki = Ki1; 
	e11_pi.Kp = Kp1; 

	e22_pi.Ki = Ki2; 
	e22_pi.Kp = Kp2; 
#endif 

	//-----------------------------------------------------------------------------------//
	// 								DOB velocity Sync 									 // 
	//-----------------------------------------------------------------------------------//
	// -500, -0.1 or -0.01
	// Kp_dobVel = -1.5
#if 0 
	v_ref = 1.0*sin(2*pi*0.1*t1); 

	dob_vel_e1 = v1_motor-v2_motor; 
	e111.i1_n =  i1_ref; 
	e111.i2_n =  i2_ref; 
	e111.w1_n = v1_motor;
	e111.w2_n = v2_motor; 

	dob_vel_e2 = v1_motor+v2_motor-2*v_ref; 
	e222.i1_n = i1_ref; 
	e222.i2_n = i2_ref; 
	e222.w1_n = v1_motor;
	e222.w2_n = v2_motor; 

	calc_DOB_vel_minus(&e111); 
	calc_DOB_vel_plus (&e222); 


	i1_ref = 0.5*((Kp_dobVel*dob_vel_e1+(J/Kt)*e111.T_n)+(Kp_dobVel*dob_vel_e2+(J/Kt)*e222.T_n)); 
	i2_ref = 0.5*(-(Kp_dobVel*dob_vel_e1+(J/Kt)*e111.T_n)+(Kp_dobVel*dob_vel_e2+(J/Kt)*e222.T_n)); 

#if 0 
	i1_ref = 0.5*((Kp_dobVel*dob_vel_e1+e111.T_n)+(Kp_dobVel*dob_vel_e2+e222.T_n)); 
	i2_ref = 0.5*(-(Kp_dobVel*dob_vel_e1+e111.T_n)+(Kp_dobVel*dob_vel_e2+e222.T_n)); 
#endif 

	T_dis_vel1 = e111.T_n; 
	T_dis_vel2 = e222.T_n; 


	if(i1_ref > 3.1){
		i1_ref = 3.1; 
	}

	if(i1_ref < -3.1){
		i1_ref = -3.1; 
	}

	if(fabs(i1_ref) <= 0.05 ){  // to nullify the zero reference current. 
		i1_ref   = 0.0;
	}


	if(i2_ref > 3.1){
		i2_ref  = 3.1; 
	}

	if(i2_ref < -3.1){
		
		i2_ref = -3.1; 
	}

	if(fabs(i2_ref) <= 0.05 ){  // to nullify the zero reference current. 
		i2_ref   = 0.0;
	}

	error_motor1 = dob_vel_e1; 
	error_motor2 = dob_vel_e2;

	DAC_data[0] = i1_ref;
	DAC_data[1] = i2_ref;

#endif 


	//-----------------------------------------------------------------------------------//
	// 								DOB theta Sync	 									 // 
	//-----------------------------------------------------------------------------------//

  //	theta_ref = 1.0*sin(2*pi*0.1*t1); 
#if 0 
	dob_theta_e1 = theta_m-theta_m2; 
	e1111.i1_n =  i1_ref; 
	e1111.i2_n =  i2_ref; 
	e1111.th1_n = theta_m;
	e1111.th2_n = theta_m2; 

	dob_theta_e2 = theta_m+theta_m2-2*theta_ref; 
	e2222.i1_n = i1_ref; 
	e2222.i2_n = i2_ref; 
	e2222.th1_n = theta_m;
	e2222.th2_n = theta_m2; 

	calc_DOB_theta_minus(&e1111); 
	calc_DOB_theta_plus (&e2222); 

	der1.x_n = Kd_theta*dob_theta_e1; 
	der2.x_n = Kd_theta*dob_theta_e2; 
#if 0
	xi_minus = (der1.y_n+(Kp_dobTheta*dob_theta_e1+(J/Kt)*e1111.T_n));
	xi_plus  = (der2.y_n+(Kp_dobTheta*dob_theta_e2+(J/Kt)*e2222.T_n)); 
#endif 

#if 0 
	xi_minus = (der1.y_n+(Kp_dobTheta*dob_theta_e1+(J/Kt)*e1111.T_n));
	xi_plus  = (der2.y_n+(Kp_dobTheta*dob_theta_e2+(J/Kt)*e2222.T_n)); 
#endif 

	xi_minus = (Kp_dobTheta*dob_theta_e1);
	xi_plus  = (Kp_dobTheta*dob_theta_e2); 

	i1_ref = 0.5*(xi_plus+xi_minus); 
	i2_ref = 0.5*(xi_plus-xi_minus); 

	if(i1_ref > 3.1){
		i1_ref  = 3.1; 
	}

	if(i1_ref < -3.1){
		
		i1_ref = -3.1; 
	}
#if 0 
	if(fabs(i1_ref) <= 0.05 ){  // to nullify the zero reference current. 
		i1_ref   = 0.0;
	}
#endif 

	if(i2_ref > 3.1){
		i2_ref  = 3.1; 
	}

	if(i2_ref < -3.1){
		
		i2_ref = -3.1; 
	}
#if 0 
	if(fabs(i2_ref) <= 0.05 ){  // to nullify the zero reference current. 
		i2_ref   = 0.0;
	}
#endif 
	error_motor1 = dob_theta_e1; 
	error_motor2 = dob_theta_e2;

	T_dis_vel1 = e1111.T_n*(J/Kt); 
	T_dis_vel2 = e2222.T_n*(J/Kt); 

#if 0 
	DAC_data[0] = i1_ref;
	DAC_data[1] = i2_ref;
#endif 
 
	DAC_data[3] = i1_ref;
	DAC_data[2] = i2_ref;

#if 0 
	DAC_data[3] = value222; // motor 1 
	DAC_data[2] = value111;	// motor 2 
#endif 
#endif 
	//------------------- LM PI testing -----------------------//

 // Ki1,2 = 1000 - 3000 Kp = 10 
	
	#if 0
	e1.error = theta_ref-theta_m2; 
	pi_calc(&e1); 
	i2_ref = e1.y_out; 
	DAC_data[2] = i2_ref; 



	e2.error = theta_ref-theta_m; 
	pi_calc(&e2); 
	i1_ref = e2.y_out; 
	DAC_data[3] = i1_ref; 


	e1.Ki = Ki1; 
	e1.Kp = Kp1; 
	e2.Ki = Ki2; 
	e2.Kp = Kp2; 
#endif 


    ds2103_out(DS2103_1_BASE,1,DAC_data);
    //start AD conversion
#if 0
    ds2003_start(DS2003_1_BASE);
    ds2003_in(DS2003_1_BASE, ADC_data);
#endif 
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

	reading.dT = DT; 


#if 0 
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

#endif 

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



