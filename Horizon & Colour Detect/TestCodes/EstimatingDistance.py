### foto retrieval
import os
import cv2
import time

folder = 'fotos/'
images = []

def split(array, nrows, ncols):
    """Split a matrix into sub-matrices."""

    r, h = array.shape
    return (array.reshape(h//nrows, nrows, -1, ncols)
                 .swapaxes(1, 2)
                 .reshape(-1, nrows, ncols))

def grid_counter(object_arrays, n_rows, n_columns):

    total = np.copy(object_arrays)

    if len(total) != 240:
        total = np.zeros((240, 520))
        for object in range(len(object_arrays)):
            total = np.add(total, object_arrays[object])

    grid_pixels = np.zeros((n_rows,n_columns))
    grid_width = 520/n_columns
    grid_heigth = 240/n_rows

    for i in range(n_rows):
        for j in range(n_columns):
            grid_pixels[i,j] = np.ndarray.sum(total[int(i*grid_heigth):int((i+1)*grid_heigth - 1), int(j*grid_width):int((j+1)*grid_width - 1)])

    grid_percentage = grid_pixels/grid_width/grid_heigth*100

    return grid_percentage

for entry in os.listdir(folder):
    images.append(cv2.imread(str(folder + entry), 0))
#     plt.imshow(image)
#     plt.show()


#### actual code working ##
# matplotlibinline
import numpy as np
import matplotlib.pyplot as plt
from scipy import ndimage as ndi
import math as mt

##timer and counter
a = time.perf_counter()
counter = 0
##

# As input to for loop list of cv2.imread('your image name')
##
for image in images[0:3]:
    counter += 1

    #     plt.imshow(image)
    #     plt.show()

    #### double gradient calc
    grad_image = np.gradient(image)
    x_grad = grad_image[0]
    y_grad = grad_image[1]

    ## x_dir
    grad_x_grad = np.gradient(x_grad)
    xx_grad = grad_x_grad[0]
    xy_grad = grad_x_grad[1]

    # y_dir
    grad_y_grad = np.gradient(y_grad)
    yy_grad = grad_y_grad[1]
    yx_grad = grad_y_grad[0]

    shape_index_matrix = np.zeros((np.shape(image)[0], np.shape(image)[1]))

    ### local curvature calc ##

    T = xx_grad + yy_grad
    D = xx_grad * yy_grad - xy_grad ** 2
    L1 = T / 2 + ((T ** 2) / 4 - D) ** (1 / 2)
    L2 = T / 2 - ((T ** 2) / 4 - D) ** (1 / 2)
    Lk1 = np.maximum(L1, L2)
    Lk2 = np.minimum(L1, L2)

    Ldiff = Lk2 - Lk1
    Ladds = Lk2 + Lk1
    Lfast_shapeindex = 2 / mt.pi * np.arctan(Ladds / Ldiff)

    hsi_m = np.zeros((np.shape(image)[0], np.shape(image)[1]))
    shape_index_matrix = Lfast_shapeindex

    ### filtering starts here
    htargets = [-0.5, .5, 0]
    hdelta = 0.05
    hmat_z_sum = []
    for htarget in htargets:
        hp_y, hp_x = np.where(np.abs(shape_index_matrix - htarget) < hdelta)
        hp_z = image[hp_y, hp_x]
        hp_z[hp_z < 100] = 0
        hmat_z = np.zeros((np.shape(image)[0] * np.shape(image)[1])).reshape(520, 240)
        for i in range(len(hp_y)):
            hmat_z[hp_x[i]][hp_y[i]] = hp_z[i]
        hmat_z_sum.append(hmat_z)

    hmat_z = hmat_z_sum[0] + hmat_z_sum[1] + hmat_z_sum[2]  # + hmat_z_sum[3]

    hmat_z = ndi.maximum_filter(hmat_z, size=(3, 3))

    fch = 15
    hmat_z[hmat_z < fch] = 0;
    hmat_z[hmat_z >= fch] = 100

    hmat_z = ndi.uniform_filter(hmat_z, size=(5, 5), mode='constant', cval=0)
    hmat_z = np.transpose(hmat_z)
    fch = 5
    hmat_z[hmat_z < fch] = 0;
    hmat_z[hmat_z >= fch] = 1
    plt.imshow(hmat_z)
    plt.show()
    ## identifying the individual objects ##
    # print(hmat_z)

    strut = ndi.generate_binary_structure(2, 10)
    label, num_label = ndi.label(hmat_z != 0, structure=strut)

    # create dictionairy that stores center point, x range, y range; of objects that are larger than sumsum > 100000
    dict_l_objects = {}
    column_count = np.zeros(5)
    grid_count = np.zeros((2,2))
    complete = np.zeros((240, 520))

    for i in range(num_label):
        object_loc_y, object_loc_x = np.where(label == i)
        # print(object_loc_y)
        try:
            z_object_points = hmat_z[object_loc_y][object_loc_x]
            # print(i, len(z_object_points))
        except:
            continue
        if sum(sum(z_object_points)) > 100000 and i != 0:

            ###########           ###
            matrix_object = np.where(label == i, 1, 0)
            # print(sum(matrix_object[75]), sum(matrix_object[200]))
            # print(hmat_z)

            # column1 = sum(map(sum,matrix_object[:,0:103]))
            column_count[0] = column_count[0] + np.ndarray.sum(matrix_object[:, 0:103])
            column_count[1] = column_count[1] + np.ndarray.sum(matrix_object[:, 104:207])
            column_count[2] = column_count[2] + np.ndarray.sum(matrix_object[:, 208:311])
            column_count[3] = column_count[3] + np.ndarray.sum(matrix_object[:, 312:415])
            column_count[4] = column_count[4] + np.ndarray.sum(matrix_object[:, 416:519])

            # print(column_count)

            complete = complete + np.copy(matrix_object)
            # print(complete)

            # a = np.array([[1, 2, 3, 4],[1, 2, 3, 4],[5, 6, 7, 8],[5, 6, 7, 8]])
            # total = np.array(matrix_object)
            # print(total)
            # lefttop, righttop, leftbottom, rightbottom = split(total, 2, 2)
            # print(a_new)
            # print(b)
            #
            # results = [[(np.ndarray.sum(matrix_object[j*120:((j+1)*120-1), k*260:((k+1)*260-1)]))
            #            for j in range(2)]
            #            for k in range(2)]
            #
            # print('result', results)
            # print((np.ndarray.sum(matrix_object[1:((j+1)*120-1), k:((k+1)*260-1)])))
            grid_count[0][0] = grid_count[0][0] + np.ndarray.sum(matrix_object[0:119, 0:259])
            grid_count[0][1] = grid_count[0][1] + np.ndarray.sum(matrix_object[0:119, 260:519])
            grid_count[1][0] = grid_count[1][0] + np.ndarray.sum(matrix_object[120:239, 0:259])
            grid_count[1][1] = grid_count[1][1] + np.ndarray.sum(matrix_object[120:239, 260:519])

            # print(grid_count)

            # print('com', ndi.measurements.center_of_mass(hmat_z, label == i))
            # ##create histogram of image
            # print(len((label == i)[0]))
            # hist_x = (label == i)[:,0]
            # hist_y = (label == i)[:,1]
            # print(len(hist_x), len(hist_y))
            #
            # clump_mass = label == i
            # # print(len(clump_mass[0]))
            #
            # ### first use integral technique on whole image
            # z_object_points = z_object_points.astype(np.uint8)
            #
            # integral_image = cv2.integral(z_object_points)
            # ##integral sum
            # sum_int = integral_image[0][0] + integral_image[-1][-1] - integral_image[-1][0] - integral_image[0][-1]
            #
            # print(sum(map(sum, clump_mass)))
            # print(sum_int, 'should be the same')

            #             print(hist_x, hist_y)
            #             H , xedges, yedges=  np.histogram2d(clump_mass_hist[0], clump_mass_hist[1])#, bins=5)#, bins=(xedges, yedges) )

            #             hist_array = ndi.measurements.histogram(hmat_z, 0, 1, 6, label==i)
            #             print(hist_array)
            #             X, Y = np.meshgrid(xedges, yedges)
            #             plt.pcolormesh(X, Y, H)
            #             plt.show()
            #######            ###

            centerpoint = (object_loc_x.mean(), object_loc_y.mean())
            print('centerpoint: ', centerpoint)
            object_x_range = (object_loc_x.min(), object_loc_x.max())
            object_y_range = (object_loc_y.max(), object_loc_y.min())
            # print('x_range midpoint:', (object_x_range[1] - object_x_range[0]) / 2 + object_x_range[0])

            ### Heuristic Distance
            height = 240 - object_y_range[0]
            # print(height)

            dict_l_objects.update({i: {'c_point': centerpoint, 'x_range': object_x_range, 'y_range': object_y_range, 'height': height}})





        #             print('centerpoint', centerpoint)
    #             print('min x-coord: ',object_loc_x.min(),'max x-coord: ', object_loc_x.max())
    #             print('lower y-coord: ', object_loc_y.max(),'upper y-coord: ', object_loc_y.min())

    for j in dict_l_objects:
        clump_mask = label == j
        #         plt.imshow(clump_mask)
        #         plt.show()
        new_image = cv2.rectangle(image, pt1=(dict_l_objects[j]['x_range'][0], dict_l_objects[j]['y_range'][1]),
                                  pt2=(dict_l_objects[j]['x_range'][1], dict_l_objects[j]['y_range'][0]),
                                  color=(255, 0, 0), thickness=3)
    plt.imshow(new_image)
    plt.vlines([103, 207, 311, 415], 0, 240)
    plt.axis([0, 519, 239, 0])
    plt.show()


    #### plotting numbers of pixels and percentage of column covered by object
    pixels_column = 520*240/5
    column_perc = column_count/pixels_column

    ## plot number of pixels
    # plt.figure()
    # plt.bar([0, 103, 207, 311, 415], column_count, width=104, align='edge')
    # plt.show()

    ## plot percentage
    # plt.figure()
    # plt.bar([0, 103, 207, 311, 415], column_perc*100, width=104, align='edge')
    # plt.show()

    ### grid format percentage
    pixels_grid = 520*240/4
    grid_perc = grid_count/pixels_grid*100
    # print(grid_perc)

    grid_percentage = grid_counter(complete, 9, 7)
    print(grid_percentage)

    # results = results / pixels_grid
    # print('result', results)
    ##plotting results

#     plt.imshow(hmat_z, interpolation='none')
#     plt.colorbar(orientation='vertical')
#     plt.show()

b = time.perf_counter()

run_time = b - a
print(run_time, 'seconds for', counter, 'images with object detection')