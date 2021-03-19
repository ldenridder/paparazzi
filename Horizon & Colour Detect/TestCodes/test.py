import numpy as np
import time

def HorizonFilter(img,m,b,buffer):
    #Crude assumption that object is rectangular
    obj_y,obj_x = np.nonzero(img)
    y1 = obj_y.min(); x1 = obj_x.min()
    y2 = obj_y.min(); x2 = obj_x.max()
    y3 = obj_y.max(); x3 = obj_x.min()
    y4 = obj_y.max(); x4 = obj_x.max()

    if(y1 < m*x1 + b + buffer and y2 < m*x2 + b + buffer and \
        y3 < m*x3 + b + buffer and y4 < m*x4 + b + buffer):
        return True #Filter out object
    elif(y1 > m*x1 + b - buffer and y2 > m*x2 + b - buffer and \
        y3 > m*x3 + b - buffer and y4 > m*x4 + b - buffer):
        return True #Filter out object
    else:
        return False #Keep object

    #More detailed check, however slow
    # x = np.transpose(np.nonzero(img))[0][1]
    # y = np.transpose(np.nonzero(img))[0][0]
    # if(y <= m*x+b):
    #     #At least one point below horizon
    #     #So check whether any point is above horizon (with buffer)
    #     below = True
    # else:
    #     #Vice versa
    #     below = False
        

    # for i in range(len(np.transpose(np.nonzero(img)))):
    #     x = np.transpose(np.nonzero(img))[i][1]
    #     y = np.transpose(np.nonzero(img))[i][0]

    #     if(below):
    #         if(y > m*x+b-buffer):
    #             return True
    #     else:
    #         if(y < m*x+b+buffer):
    #             return True
    # return False

########################
#For testing
########################
A = np.zeros((240,520))
for i in range(150,200):
    for j in range(200,250):
        A[i][j] = 1

m = 0; b = 120
buffer = 3
startTime = time.time()
print(HorizonFilter(A,m,b,buffer))
endTime = time.time()
print(endTime-startTime)
