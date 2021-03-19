import math

def horizon(theta,phi,Y,pitchGain):
    #Theta and phi given in degrees
    #X and Y given in pixels

    #Note: A positive slope is down and to the right in the image
    #Make sure that positive phi is a roll to the right

    #The pitchGain is a proportional gain which relates the pitch angle to a pixel translation

    m = math.tan(math.radians(phi))
    b = (Y/2) + pitchGain*theta
