#include "lowPassDer.h"

void calcDerivative(struct derivative *der){

    der-> y_n = (2-der->g*der->T)/(2+der->g*der->T)*(der->y_n_1)+ (2)/(2+der->g*der->T)*(der->x_n - der->x_n_1); 
    der->y_n_1 = der->y_n; 
    der->x_n_1 = der->x_n; 
    
}