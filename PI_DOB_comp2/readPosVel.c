#include "readPosVel.h"

void measurePositonVel(struct readPosVel *read){

    long count1,count2,len1,len2; 
    dsfloat freq1,freq2; 
    
	read->position1 = ds3001_read_position(DS3001_1_BASE,1)*100; 
	


	if(read->position1 > read->position1_prev ){

		read->dir1 = 1; 
	}
	else if(read->position1 < read->position1_prev ){

		read->dir1 = -1; 
	}

	read->position1_prev = read->position1; 



    count1 = 35; 
	ds4002_f2d_overl (DS4002_1_BASE, 1, count1, &len1, &freq1);  // calculates frequency
	read->vel1 = read->dir1*freq1/44000;  // reads both rising and falling edges (1/2 -> 22*2kHz) 
	read->position2 = ds3001_read_position(DS3001_2_BASE,2)*100; 
	

	if(read->position2 >=  read->position2_prev ){
		read->dir2 = 1; 
	}
	else {
		read->dir2 = -1; 
	}


	read->position2_prev = read->position2; 

	count2 = 35; 
	ds4002_f2d_overl (DS4002_1_BASE, 2, count2, &len2, &freq2);  // calculates frequency
	read->vel2 = read->dir2*freq2/44000;  // reads both rising and falling edges (1/2 -> 22*2kHz) 


}


void measurePositonVel2(struct readPosVel *read){
    
	// 
	// reading position 1 and vel 1   
	//
	read->position1 = ds3001_read_position(DS3001_1_BASE,4)*100; 
	read->vel1 = (read->position1-read->position1_prev)/(read->dT);   // reads both rising and falling edges (1/2 -> 22*2kHz) 
	read->position1_prev = read->position1;

	// 
	// reading position 2 and vel 2
	//

	read->position2 = ds3001_read_position(DS3001_2_BASE,2)*100; 	
	read->vel2 = (read->position2-read->position2_prev)/(read->dT);   // reads both rising and falling edges (1/2 -> 22*2kHz) 
	read->position2_prev = read->position2;


}