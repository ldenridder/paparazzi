#ifndef HORIZON_H
#define HORIZON_H


class Horizon
{
private:
    //Instead of functions that return a value, change private 
    // variable within the function
    int im;
    int m;
    int b;
    
    int n_s;
    int mu_s;

    int n_g;
    int mu_g;

    int dist;
 
    //More to be added


public:
    Horizon(int im, int m, int b, );

    void meanBGR_sky();
    void covSky();

    void meanBGR_ground();
    void covGround();
    
    void findHorizon();
    void filter_colour();

};
 
#endif