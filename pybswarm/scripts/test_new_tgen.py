#%%
import inspect
import sys
import os
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
root_dir = os.path.join(path, os.path.pardir)
sys.path.insert(0, root_dir)

import bswarm
import bswarm.traj_gen_new as tgen
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

#%%
n_drone = 1
pi = np.pi

#read waypoints (single circular track)
#waypoints
# circle back
waypoints = np.array([
    [0,0,0],
    [1,0,0],
    [1,1,0],
    [0,1,0],
    [0,0,0],
]).T

# #straight line
# waypoints = np.array([
#     [0,0,0],
#     [0,1,0],
#     [0,2,0],
#     [0,3,0],
#     [0,4,0],
# ]).T

ax = plt.axes(projection='3d')
ax.plot3D(waypoints[0,:], waypoints[1,:], waypoints[2,:], 'ro')
print(waypoints.shape)
#%%
T = np.zeros((waypoints.shape[1]-1))
for i in range(len(T)):
    T[i] = (i+1)*(i+1)

print(T)

planner = tgen.plan_min_snap
trajx = planner(waypoints[0,:],T)
trajy = planner(waypoints[1,:],T)
trajz = planner(waypoints[2,:],T)
cx = trajx.c
cy = trajy.c
cz = trajz.c
cyaw = [0]*8
traj=[]
for leg in range(len(T)):
    cxi = np.flip(cx[8*leg:8*(leg+1)],0)
    cyi = np.flip(cy[8*leg:8*(leg+1)],0)
    czi = np.flip(cz[8*leg:8*(leg+1)],0)
    traj.append(np.hstack([T[leg],cxi,cyi,czi,cyaw]))
traj = np.array(traj)
p = np.array(
    [trajx.compute_trajectory()['x'],
    trajy.compute_trajectory()['x'],
    trajz.compute_trajectory()['x']
])

plt.figure()
plt.grid()
ax = plt.axes(projection='3d')

ax.plot3D(p[0,:],p[1,:],p[2,:])

ax.plot3D(waypoints[0,:],waypoints[1,:],waypoints[2,:],'ro')

plt.figure()
plt.title('x')
trajx.plot()
plt.figure()
plt.title('y')
trajy.plot()
plt.figure()
plt.title('z')
trajz.plot()
plt.show()
#%%
# import json
# with open('json/test.json', 'w') as f:
#     json.dump(formation, f)

# print('start printing leg')
# for leg in traj:
#     T = leg[0]
#     print(leg)
    
#%%
header = 'duration,x^0,x^1,x^2,x^3,x^4,x^5,x^6,x^7,y^0,y^1,y^2,y^3,y^4,y^5,y^6,y^7,z^0,z^1,z^2,z^3,z^4,z^5,z^6,z^7,yaw^0,yaw^1,yaw^2,yaw^3,yaw^4,yaw^5,yaw^6,yaw^7'
np.savetxt('csv/test1.csv',traj,delimiter=',',header=header)
#%%
