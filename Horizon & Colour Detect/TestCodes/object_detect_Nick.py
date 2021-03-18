##### start list of functions ######

def gradient_image(image):
        grads_image = np.gradient(image)
        return np.gradient(grads_image[0])[0], np.gradient(grads_image[1])[1], np.gradient(grads_image[0])[1]

def shape_ind(xx_grad,yy_grad, xy_grad):
        T = xx_grad+yy_grad
        D = xx_grad*yy_grad - xy_grad**2
        L1 = T/2 + ((T**2)/4-D)**(1/2)
        L2 = T/2 - ((T**2)/4-D)**(1/2)
        Lk1 = np.maximum(L1,L2)
        Lk2 = np.minimum(L1,L2)
        Ldiff = Lk2-Lk1
        Ladds = Lk2+Lk1
        shape_index_matrix = 2/mt.pi*np.arctan(Ladds/Ldiff)
        return shape_index_matrix   

def hmat_z_func(htarget): 
        hdelta=0.05
        hp_y, hp_x = np.where(np.abs(shape_index_matrix - htarget) < hdelta)
        hp_z = image[hp_y, hp_x]
        hp_z[hp_z<100] = 0
        hmat_z = np.zeros((np.shape(image)[0]* np.shape(image)[1])).reshape(520,240)
        for i in zip(hp_y,hp_x,hp_z):
            hmat_z[i[1],i[0]] = i[2]        
        return hmat_z
    
def filt_noise(hmat_z):
        hmat_z = ndi.maximum_filter(hmat_z, size=(3,3))
        fch = 50
        hmat_z[hmat_z<fch] = int(0);
        hmat_z[hmat_z>=fch] = int(100)
        hmat_z = ndi.uniform_filter(hmat_z, size=(5,5), mode='constant', cval= 0)
        fch=5
        hmat_z[hmat_z<fch] = int(0);
        hmat_z[hmat_z>=fch] = int(1)
        return np.transpose(hmat_z)

    
## below is a function that creates coordinates used in the sum calc for the integral image per grid box 
### create top left top right bottom left bottom right coordinates of the grid box of interest
def create_grid_boxes(num_col, num_rows):
    num_col = num_col
    num_rows = num_rows
    box_x_size = 520/num_col
    box_y_size = 240/num_rows

        # 4 points for every box ## 
    points_list = []
    for n in range(num_col):
        for k in range(num_rows):
            TL_x =  n*box_x_size
            TL_y =  k*box_y_size 
            BL_x = TL_x
            BL_y = TL_y + box_y_size
            BR_x = BL_x + box_x_size
            BR_y = BL_y 
            TR_x = TL_x + box_x_size
            TR_y = TL_y 
            points_string = [int(TL_x),int(TL_y)],[int(BL_x),int(BL_y)],[int(BR_x), int(BR_y)],[int(TR_x),int(TR_y)]
            points_list.append(points_string)
    return points_list

####### end list of function #######




######  imports and foto retrieval ######

import os
import cv2
import time
import seaborn
import numpy as np
import matplotlib.pyplot as plt
from scipy import ndimage as ndi
import math as mt

folder = 'fotos/'
images= []

for entry in os.listdir(folder):
    images.append(cv2.imread(str(folder+entry),0))

###### end of import and retrieval #######


###### start of running code ######

a = time.perf_counter()
counter = 0
for image in images:  
#     plt.imshow(image)
#     plt.show()
    counter+=1
    
    ## hmat_z produced
    xx_grad, yy_grad, xy_grad = gradient_image(image)

    shape_index_matrix = shape_ind(xx_grad, yy_grad, xy_grad)

    hmat_z =  hmat_z_func(-0.5) + hmat_z_func(.5) + hmat_z_func(0)

    hmat_z = filt_noise(hmat_z)
    ## identifying the individual objects ##
    strut = ndi.generate_binary_structure(2,10)
    label, num_label = ndi.label(hmat_z!=0, structure = strut)
    ## pasting item info in dict and list
    dict_l_objects = {}
    two_d_arr_sum = 0
    
    ### at this a merge with Hani's code
    num_col = 10
    num_rows= 10
    points_list = create_grid_boxes(num_col,num_rows)
    for i in range(num_label):
        object_loc_y, object_loc_x = np.where(label==i)
        clump_mass = label == i
        clump_mass = clump_mass.astype(int)
        clump_mass = clump_mass.astype(np.uint8)
        integral_image_clump = cv2.integral(clump_mass)
        sum_int_clump = integral_image_clump[1][1] + integral_image_clump[-1][-1] -integral_image_clump[-1][1] -integral_image_clump[1][-1]
        print(sum_int_clump)
        
        ## this is a mockup for the horizon conditional
        COM_y = ndi.measurements.center_of_mass(hmat_z, label==i)[0]
        
        # for loop that excludes objects with certain size: sum_int_clump and location wrt COM_y
        if sum_int_clump >1000 and i!=0 and 50<COM_y<200:
            
            ## obtaining pixel count per grid box per image into an 1d array with length of n_rows*n_col
            sum_per_square = [integral_image_clump[points_list[x][0][1]][points_list[x][0][0]] + integral_image_clump[points_list[x][2][1]][points_list[x][2][0]] - integral_image_clump[points_list[x][1][1]][points_list[x][1][0]] - integral_image_clump[points_list[x][3][1]][points_list[x][3][0]] for x in range(len(points_list))]
            ## creating a 2d array similar to the grid box division of the image
            two_d_arr = np.transpose(np.reshape(sum_per_square, (num_col,num_rows)))
            ## adding all the individual 2d array per object of interest into 1 2d array w/grid box
            two_d_arr_sum += two_d_arr
            
            ## determine individual object features and appending them to dictionairy 
            centerpoint_mean = (object_loc_x.mean(), object_loc_y.mean())
            object_x_range = (object_loc_x.min(),object_loc_x.max())
            object_y_range = (object_loc_y.max(), object_loc_y.min())
            dict_l_objects.update({i:{'c_point':centerpoint_mean,'x_range':object_x_range, 'y_range': object_y_range}})      

    ### this is the heatmap plot, you can use vmax as a measure of density per grid box, leaving it out gives the real range of density        
    try:
        seaborn.heatmap(two_d_arr_sum, vmin=0, vmax=100) #,vmax=800)
        plt.show()
    except:
        continue
        
## counter and timer calc.
    
b = time.perf_counter()

run_time = b-a
print(run_time, 'seconds for', counter, 'images with object detection, grid construction and plotting(with slow ass seaborn)')    


#### end of running code #######


