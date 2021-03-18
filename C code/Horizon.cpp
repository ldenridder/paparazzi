#include "Horizon.h"

//Include needed packages here


Horizon::Horizon()
{
    meanBGR_sky();
    covSky();

    meanBGR_ground();
    covGround();
    
    findHorizon();
    filter_colour();
}

void Horizon::meanBGR_ground()
{
    /* //Python code
    n_s = 0
    mu_s = [0,0,0]
    for x in range(im.shape[1]):
        for y in range(im.shape[0]):
            if(y<m*x+b):
                #Pixel is in sky for this 'test' horizon line
                n_s += 1
                mu_s += im[y,x,:]
    if(n_s == 0):
        mu_s = 0
    else:
        mu_s = mu_s/n_s
    */
}

void Horizon::meanBGR_sky()
{
/*
    n_g = 0
    mu_g = 0
    for x in range(im.shape[1]):
        for y in range(im.shape[0]):
            if(y>=m*x+b):
                #Pixel is in ground for this 'test' horizon line
                n_g += 1
                mu_g += im[y,x,:]
    if (n_g == 0):
        mu_g = 0
    else:
        mu_g = mu_g/n_g
*/
}

void Horizon::covSky()
{
/*
    #Can be improved by extracting all pixels above and below
    #horizon line once, instead of in each function
    sigma_s = np.zeros((3,3))
    for x in range(im.shape[1]):
        for y in range(im.shape[0]):
            if(y<m*x+b):
                diff = im[y,x,:] - mu_s
                sigma_s += np.outer(diff,diff)
    sigma_s = sigma_s / (n_s-1)
*/
}

void Horizon::covGround()
{
/*
    #Can be improved by extracting all pixels above and below
    #horizon line once, instead of in each function
    sigma_g = np.zeros((3,3))
    for x in range(im.shape[1]):
        for y in range(im.shape[0]):
            if(y>=m*x+b):
                diff = im[y,x,:] - mu_g
                sigma_g += np.outer(diff,diff)
    sigma_g = sigma_g / (n_g-1)
*/
}

void Horizon::findHorizon()
{
/*

*/
}

void Horizon::filter_colour()
{
/*

*/
}