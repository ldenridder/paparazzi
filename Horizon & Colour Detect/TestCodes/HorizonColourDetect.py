import numpy as np
import cv2
from skimage.measure import block_reduce
import matplotlib.pyplot as plt
import time

def meanBGR_sky(im,m,b):
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
    return n_s, mu_s

def meanBGR_ground(im,m,b):
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
    return n_g, mu_g

def covSky(im,m,b,n_s,mu_s):
    #Can be improved by extracting all pixels above and below
    #horizon line once, instead of in each function
    sigma_s = np.zeros((3,3))
    for x in range(im.shape[1]):
        for y in range(im.shape[0]):
            if(y<m*x+b):
                diff = im[y,x,:] - mu_s
                sigma_s += np.outer(diff,diff)
    sigma_s = sigma_s / (n_s-1)
    return sigma_s

def covGround(im,m,b,n_g,mu_g):
    #Can be improved by extracting all pixels above and below
    #horizon line once, instead of in each function
    sigma_g = np.zeros((3,3))
    for x in range(im.shape[1]):
        for y in range(im.shape[0]):
            if(y>=m*x+b):
                diff = im[y,x,:] - mu_g
                sigma_g += np.outer(diff,diff)
    sigma_g = sigma_g / (n_g-1)
    return sigma_g

def findHorizon(im,n1,n2,Y,X):
    #Loop through values of m and b
    #Compute each respective J1 (or J2)
    #Take m and b which maximize J1 (or J2)
    m_max = Y/X
    #m_vec_neg = np.linspace(-m_max,0,n1/2)
    #m_vec_pos = np.linspace(0,m_max,n1/2)
    m_vec_pos = [0]
    m_vec_neg = [0]
    #b_vec_upper = np.linspace(0,Y/2,n2/2)
    #b_vec_lower = np.linspace(Y/2,Y,n2/2)
    b_vec_upper = np.linspace(Y/2-Y/8,Y/2,n2/2)
    b_vec_lower = np.linspace(Y/2,Y/2+Y/8,n2/2)
    J_max = 0
    m_horizon = 0
    b_horizon = 0

    #Note that positive slope is down and to the right
    #for m in m_vec_pos:
    m = m_vec_neg[0]
    for b in b_vec_upper:
        if(b==0 and m==0):
            continue
        n_s,mu_s = meanBGR_sky(im,m,b)
        n_g,mu_g = meanBGR_ground(im,m,b)
        if (n_s == 0 or n_g == 0):
            continue
        sigma_s = covSky(im,m,b,n_s,mu_s)
        sigma_g = covGround(im,m,b,n_g,mu_g)
        eig_s1 = np.linalg.eig(sigma_s)[0][0]
        eig_s2 = np.linalg.eig(sigma_s)[0][1]
        eig_s3 = np.linalg.eig(sigma_s)[0][2]
        sigma_s = covSky(im,m,b,n_s,mu_s)
        sigma_g = covGround(im,m,b,n_g,mu_g)
        eig_g1 = np.linalg.eig(sigma_g)[0][0]
        eig_g2 = np.linalg.eig(sigma_g)[0][1]
        eig_g3 = np.linalg.eig(sigma_g)[0][2]
        det_s = np.linalg.det(sigma_s)
        det_g = np.linalg.det(sigma_g)
        #J1 = 1/(np.linalg.det(sigma_s)+np.linalg.det(sigma_g))
        J2 = 1/(det_s + det_g + (eig_s1+eig_s2+eig_s3)**2 + (eig_g1+eig_g2+eig_g3)**2)
        if (J2 > J_max):
            m_horizon = m; b_horizon = b
            J_max = J2
            mu_sFINAL = mu_s
            mu_gFINAL = mu_g
    #for m in m_vec_neg:
    for b in b_vec_lower:
        if(b==Y and m==0):
            continue
        n_s,mu_s = meanBGR_sky(im,m,b)
        n_g,mu_g = meanBGR_ground(im,m,b)
        sigma_s = covSky(im,m,b,n_s,mu_s)
        sigma_g = covGround(im,m,b,n_g,mu_g)
        eig_s1 = np.linalg.eig(sigma_s)[0][0]
        eig_s2 = np.linalg.eig(sigma_s)[0][1]
        eig_s3 = np.linalg.eig(sigma_s)[0][2]
        eig_g1 = np.linalg.eig(sigma_g)[0][0]
        eig_g2 = np.linalg.eig(sigma_g)[0][1]
        eig_g3 = np.linalg.eig(sigma_g)[0][2]
        det_s = np.linalg.det(sigma_s)
        det_g = np.linalg.det(sigma_g)
        #J1 = 1/(np.linalg.det(sigma_s)+np.linalg.det(sigma_g))
        J2 = 1/(det_s + det_g + (eig_s1+eig_s2+eig_s3)**2 + (eig_g1+eig_g2+eig_g3)**2)
        if (J2 > J_max):
            m_horizon = m; b_horizon = b
            J_max = J2
            mu_sFINAL = mu_s
            mu_gFINAL = mu_g
    return m_horizon,b_horizon,J_max,mu_sFINAL,mu_gFINAL

def filter_colour(im,resize_factor,m,b,mu_s,mu_g,threshold_s,threshold_g):
    im = cv2.resize(im, (int(im.shape[1]/resize_factor),int(im.shape[0]/resize_factor)))
    Filtered = np.ones([im.shape[0],im.shape[1]])
    y_low_sky = mu_s[0] - threshold_s; y_high_sky = mu_s[0] + threshold_s
    u_low_sky = mu_s[1] - threshold_s; u_high_sky = mu_s[1] + threshold_s
    v_low_sky = mu_s[2] - threshold_s; v_high_sky = mu_s[2] + threshold_s
    y_low_ground = mu_g[0] - threshold_g; y_high_ground = mu_g[0] + threshold_g
    u_low_ground = mu_g[1] - threshold_g; u_high_ground = mu_g[1] + threshold_g
    v_low_ground = mu_g[2] - threshold_g; v_high_ground = mu_g[2] + threshold_g

    for y in range(im.shape[0]):
        for x in range(im.shape[1]):
            if (y<m*x+b): #Sky
                if(im[y,x,0] >= y_low_sky and im[y,x,0] <= y_high_sky and \
                    im[y,x,1] >= u_low_sky and im[y,x,1] <= u_high_sky and \
                    im[y,x,2] >= v_low_sky and im[y,x,2] <= v_high_sky):
                        Filtered[y,x] = 0
            else: #Ground
                if(im[y,x,0] >= y_low_ground and im[y,x,0] <= y_high_ground and \
                    im[y,x,1] >= u_low_ground and im[y,x,1] <= u_high_ground and \
                    im[y,x,2] >= v_low_ground and im[y,x,2] <= v_high_ground):
                        Filtered[y,x] = 0

    return Filtered #Returns array where cell is 1 if colour found


#####################
#MAIN CODE
#####################
im = cv2.imread('test2.jpg')
imYUV = cv2.cvtColor(im,cv2.COLOR_BGR2YUV)
ar = np.asarray(im)
print(ar.shape) #[y, x, 3 denotes YUV]

#Scale down by factors of 4. For memory considerations
im_reduced = block_reduce(im,block_size = (10,10,1),func=np.mean)
ar_reduced = np.asarray(im_reduced)
print(ar_reduced.shape)

start = time.time()
m,b,J_max,mu_s,mu_g = findHorizon(ar_reduced,40,40,ar_reduced.shape[0],ar_reduced.shape[1])
end = time.time()
print('m = %s' %m)
print('b = %s' %b)
print('J_max = %s' %J_max)

threshold_s = 20
threshold_g = 20


Filtered = filter_colour(im_reduced,1,m,b,mu_s,mu_g,threshold_s,threshold_g)

runTime = end - start
print('Run Time = %s' %runTime)

b = b #Image was scaled down by 4
x = [0,im_reduced.shape[1]]
y = [b,m*x[1]+b]
plt.figure()
plt.imshow(Filtered)
plt.plot(x,y,color="red",linewidth=3)
plt.show()

            


