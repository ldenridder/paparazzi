######  imports and image retrieval ######

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
filename = []

for entry in os.listdir(folder):
    x = cv2.imread(str(folder+entry),0)
    images.append(x)#cv2.rotate(x, cv2.ROTATE_90_COUNTERCLOCKWISE))
    filename.append(entry)

###### end of import and retrieval #######


##### start list of functions ######

def gradient_image(image):
    grads_image = np.gradient(image)
    return np.gradient(grads_image[0])[0], np.gradient(grads_image[1])[1], np.gradient(grads_image[0])[1]

def shape_ind(xx_grad, yy_grad, xy_grad):
    T = xx_grad + yy_grad
    D = xx_grad * yy_grad - xy_grad ** 2
    L1 = T / 2 + ((T ** 2) / 4 - D) ** (1 / 2)
    L2 = T / 2 - ((T ** 2) / 4 - D) ** (1 / 2)
    Lk1 = np.maximum(L1, L2)
    Lk2 = np.minimum(L1, L2)
    Ldiff = Lk2 - Lk1
    Ladds = Lk2 + Lk1
    shape_index_matrix = 2 / mt.pi * np.arctan(Ladds / Ldiff)
    return shape_index_matrix

def hmat_z_func(htarget):
    hdelta = 0.05
    hp_y, hp_x = np.where(np.abs(shape_index_matrix - htarget) < hdelta)
    hp_z = image[hp_y, hp_x]
    hp_z[hp_z < 100] = 0
    hmat_z = np.zeros((np.shape(image)[0] * np.shape(image)[1])).reshape(520, 240)
    for i in zip(hp_y, hp_x, hp_z):
        hmat_z[i[1], i[0]] = i[2]
    return hmat_z

def noise_filter(hmat_z):
    hmat_z = ndi.maximum_filter(hmat_z, size=(3, 3))
    fch = 50
    hmat_z[hmat_z < fch] = int(0);
    hmat_z[hmat_z >= fch] = int(100)

    hmat_z = ndi.uniform_filter(hmat_z, size=(5, 5), mode='constant', cval=0)
    fch = 5
    hmat_z[hmat_z < fch] = int(0);
    hmat_z[hmat_z >= fch] = int(1)
    return np.transpose(hmat_z)

def sum_mass(label, i):
    clump_mass = label == i
    clump_mass = clump_mass.astype(int)
    clump_mass = clump_mass.astype(np.uint8)

    integral_image_clump = cv2.integral(clump_mass)

    sum_int_clump = integral_image_clump[1][1] + integral_image_clump[-1][-1] - integral_image_clump[-1][1] - \
                    integral_image_clump[1][-1]

    return sum_int_clump

def horizon(theta,phi,Y,pitchGain):
    #Theta and phi given in degrees
    #X and Y given in pixels

    #Note: A positive slope is down and to the right in the image
    #Make sure that positive phi is a roll to the right

    #The pitchGain is a proportional gain which relates the pitch angle to a pixel translation

    m = mt.tan(mt.radians(phi))
    b = (Y/2) + pitchGain*theta

    return m, b

def horizon_filter(object_x_range,object_y_range,m,b,buffer):
    #Crude assumption that object is rectangular
    #obj_y,obj_x = np.nonzero(img)
    y1 = object_y_range[1]; x1 = object_x_range[0]
    y2 = object_y_range[1]; x2 = object_x_range[1]
    y3 = object_y_range[0]; x3 = object_x_range[0]
    y4 = object_y_range[0]; x4 = object_x_range[1]

    if(y1 < m*x1 + b - buffer and y2 < m*x2 + b - buffer and \
        y3 < m*x3 + b - buffer and y4 < m*x4 + b - buffer):

        return True #Filter out object

    elif(y1 > m*x1 + b + buffer and y2 > m*x2 + b + buffer and \
        y3 > m*x3 + b + buffer and y4 > m*x4 + b + buffer):

        return True #Filter out object

    else:
        return False #Keep object

def grid_counter(all_objects, n_rows, n_columns):

    grid_pixels = np.zeros((n_rows,n_columns))
    grid_width = 520/n_columns
    grid_heigth = 240/n_rows

    for i in range(n_rows):
        for j in range(n_columns):
            grid_pixels[i,j] = np.ndarray.sum(all_objects[int(i*grid_heigth):int((i+1)*grid_heigth - 1), int(j*grid_width):int((j+1)*grid_width - 1)])

    # grid_percentage = grid_pixels/grid_width/grid_heigth*100

    return grid_pixels

def output_conversion(grid_percentage):
    nav_input = 7*[0]

    for i in range(len(grid_percentage[0])):
        for j in range(len(grid_percentage)-1, -1, -1):
            if grid_percentage[j][i] == 0.:
                nav_input[i] += 1

    return nav_input

####### end list of function #######


###### start of running code ######

a = time.perf_counter()
counter = 0

for image in images:
    # plt.imshow(image)
    # plt.show()

    counter += 1

    ## Hessian of Z matrix produced
    xx_grad, yy_grad, xy_grad = gradient_image(image)
    shape_index_matrix = shape_ind(xx_grad, yy_grad, xy_grad)
    # print(shape_index_matrix)
    # print(len(shape_index_matri/x), len(shape_index_matrix[0]))

    ## Calculate Hessian matrix and filter on noise
    hmat_z = hmat_z_func(-0.5) + hmat_z_func(.5) + hmat_z_func(0)
    # print(hmat_z[21])
    hmat_z = noise_filter(hmat_z)

    ## Identifying the individual objects
    strut = ndi.generate_binary_structure(2, 10)
    label, n_label = ndi.label(hmat_z != 0, structure=strut)

    ## Initiate dictionary and list to copy information in
    dict_l_objects = {}
    two_d_arr_sum = 0

    ## Set number of rows and columns for grid, and initiate a 2d array to count number of pixels
    n_columns = 7
    n_rows = 9
    all_objects = np.zeros((240,520))

    ## Get horizon from states and set buffer value for horizon
    m, b = horizon(0, 0, 240, 1) # theta, phi, Y, pitchgain
    buffer = 10 # in pixels

    ## Loop through all objects
    for i in range(1, n_label+1):

        ## Determine x- and y-locations of objects
        object_loc_y, object_loc_x = np.where(label == i)

        object_x_range = (object_loc_x.min(), object_loc_x.max())
        object_y_range = (object_loc_y.max(), object_loc_y.min())

        ## Sum all pixels corresponding to the object
        sum_int_clump = sum_mass(label, i)

        ## For loop that excludes objects that are smaller than certain size and do not cross the horizon
        if sum_int_clump > 1000 and horizon_filter(object_x_range,object_y_range,m,b,buffer) == False:

            ## Include object in all objects and append information to dictionary
            all_objects += np.where(label == i, 1, 0)
            dict_l_objects.update({i: {'x_range': object_x_range, 'y_range': object_y_range}})

    ## Find the percentage of pixels covered by object in the grid
    grid_percentage = grid_counter(all_objects, n_rows, n_columns)
    nav_input = output_conversion(grid_percentage)
    print(filename[counter-1], nav_input)

########## Only to check images! Not to implement in C! ############
    for j in dict_l_objects:
        new_image = cv2.rectangle(image, pt1=(dict_l_objects[j]['x_range'][0], dict_l_objects[j]['y_range'][1]),
                                  pt2=(dict_l_objects[j]['x_range'][1], dict_l_objects[j]['y_range'][0]),
                                  color=(255, 0, 0), thickness=3)

    if len(dict_l_objects) != 0:
        plt.clf()
        plt.imshow(new_image)
        plt.title(str(filename[counter-1]))
        plt.vlines([74, 148, 222, 296, 370, 444], 0, 240)
        plt.hlines([27, 54, 81, 109, 136, 163, 190, 217], 0, 520)
        plt.axis([0, 519, 239, 0])
        plt.show()

    else:
        plt.clf()
        plt.imshow(image)
        plt.title(str(filename[counter - 1]))
        plt.vlines([74, 148, 222, 296, 370, 444], 0, 240)
        plt.hlines([27, 54, 81, 109, 136, 163, 190, 217], 0, 520)
        plt.axis([0, 519, 239, 0])
        plt.show()

########### Visual check ends here ###################

## Counter and timer calculation
b = time.perf_counter()
run_time = b - a

print(run_time, 'seconds for', counter,
      'images with object detection, grid construction and plotting')

#### end of running code #######

