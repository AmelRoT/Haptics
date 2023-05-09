struct readPosVel {

    dsfloat position1; 
    dsfloat position1_prev; 
    dsfloat dir1; 
    dsfloat vel1; 
    dsfloat position2; 
    dsfloat position2_prev; 
    dsfloat dir2; 
    dsfloat vel2; 

}; 

#define defaultRead { 0,0,\
0,0,0,0,0,0}

void measurePositonVel(struct readPosVel *read); 


/// @param position1      - theta measured of the first motor 
/// @param position1_prev - previous theta measured of the first motor 
/// @param dir1 -           direction measured of the first motor 
/// @param vel1 -           velocity measured of the first motor 
/// @param position2 -      theta measured of the 2nd motor 
/// @param position2_prev - previous theta measured of the first motor 
/// @param dir2 -           direction measured of the 2nd motor
/// @param vel2 -           velocity measured of the 2nd motor 
