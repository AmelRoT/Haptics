dsfloat calcFriction(dsfloat w){

    dsfloat force_friction; 
   // force_friction = 0.025*sign(w)+2.2*w; // k = 2.2  
    force_friction = 0.025*sign(w)+0.1*w;

    return force_friction; 

}