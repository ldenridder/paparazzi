import math
import time





def vel_function(vx,vy):
  velocity = math.sqrt(vx*vx + vy*vy)
  return velocity

# A negative distance means the drone is moving away from the object
def distance_function(obj_width1, obj_width2, time_lapse,velocity):
  if type(obj_width1 == int):
      obj_width1 = float(obj_width1)
      
  if type(obj_width2 == int):
      obj_width2 = float(obj_width2)
      
  if type(velocity == int):
      velocity = float(velocity)

  
  if type(time_lapse == int):
      time_lapse = float(time_lapse)
      
  dis_trav = velocity*time_lapse
  
  if (obj_width1 > obj_width2):
      d = dis_trav + dis_trav/(obj_width1/obj_width2 -1) 
      obj_dist = d
  elif (obj_width1 < obj_width2):
      d = dis_trav/(1.0 - obj_width1/obj_width2)
      obj_dist = d-dis_trav
  else:
      print("no distance estimation can be done at this moment")
      obj_dist = float("nan")
       
  return obj_dist

t0 = time.time()
print(distance_function(5,8,0.1,5))
#print(vel_function(5,5))
t1 = time.time()

total = t1-t0

print (total)
