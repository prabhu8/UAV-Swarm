import traj_gen as traj 
import numpy as np
import matplotlib.pyplot as plt
import math
import csv

""" Enter your desired values in these variables"""

"""THIS WORKS BEST FOR DRONES BETWEEN 7-14 DRONES"""
drones = 9

highest = 1.6           # Highest drone       
ground_height = 0.3     # Minimum heigh off the ground        
m = .8                  # Recommended gradient of the Purdue P 

""" Number of drones per shape broken into 3 parts of the P"""

P_height = highest - ground_height      # Total height of P 
circ_drone = round(drones/2)            # Circular part of P 
ext_drone = math.floor(circ_drone/4)    # Drones used for connect semi-circle to diagonal

rad = P_height/3        # Size of the radius of the semi-circle 
z = []
x = []
ang = np.pi/2       

add_h = highest - rad

for i in range(int(circ_drone)):
    inc = np.pi/(circ_drone-1)
    
    x.append(rad*np.cos(ang))
    z.append(rad*np.sin(ang)+add_h)
    ang -= inc

for i in range(len(x)):
    x[i] += rad/4

intercept = highest/m
start_ext = min(x)
spacing = abs(((min(z) - intercept)/m - min(x))/(ext_drone+1))

for i in range(int(ext_drone)):
    z.append(highest-2*rad)
    x.append(start_ext-spacing*(i+1))

diag_drone = drones - circ_drone - ext_drone 


for i in range(int(diag_drone)):
    z.append(highest-(P_height/(diag_drone-1)*i))
    x.append((z[i+int(drones-diag_drone)]-intercept)/m)

center = (max(x)-min(x))/3

for i in range(len(x)):
    x[i] = x[i] + center


z,x = zip(*sorted(zip(z,x)))
# z, x = (list(t) for t in zip(*sorted(zip(z, x))))
y00 = len(x)
ym = .3


y = []
for i in range(len(x)):
    y.append (ym*z[i])

center = (max(y) - min(y))/3

for i in range(len(y)):
    y[i] = y[i] - center

# Clears the csv file
filename = "PurdueP.csv"
f = open(filename,"w+")
f.close 

# Writes the new data
with open(filename, 'w') as f:
    writer = csv.writer(f)
    
    for i in range(len(x)):
        writer.writerow([x[i],y[i],z[i]])

rangez = (max(z)-min(z))/2

plt.figure()
# plt.axis([min(x), max(x), min(), max(z), min(z), max(z)])
ax = plt.axes(projection='3d')
ax.scatter3D(x,y,z)
plt.show()
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')