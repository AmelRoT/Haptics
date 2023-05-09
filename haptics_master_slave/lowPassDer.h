

struct derivative {

    dsfloat T; 
    dsfloat g; 
    dsfloat x_n; 
    dsfloat x_n_1; 
    dsfloat y_n; 
    dsfloat y_n_1; 

}; 

#define defaultValuesDer{ 1e-5,100,0,0,\
0,0}


void calcDerivative(struct derivative *der); 
