#%%
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import bswarm
import bswarm.trajectory_generation as tgen 

#%%
n_drone = 7
pi = np.pi
h = [0.3,0.5,0.7,0.9]
print(h)
#read wayponts
#init pos
Pi = []
for hi in h:
    Pi.append(np.array([1.0, 0, hi+1]).T)
Pi = np.array(Pi)
print(Pi)
ax = plt.axes(projection='3d')
ax.plot3D(Pi[:,0], Pi[:,1], Pi[:,2], 'ro')
#%%
#spiral-in path:
waypoints = []
for hi in range(len(h)):
    r = np.linspace(1,0,10)
    Pwi = [np.array([1.0, 0.0, 1.0])]
    Pwi.append(Pi[hi])
    for i in range(len(r)):
        theta = i*2*pi/len(r)
        print(r[i])
        x = r[i]*np.cos(theta)
        y = r[i]*np.sin(theta)
        z = h[hi]+1
        Pwi.append(np.array([x,y,z]))
    Pwi.append(Pi[hi])
    waypoints.append(Pwi)
waypoints = np.array(waypoints)

#print(waypoints)
print(waypoints.shape)
ax = plt.axes(projection='3d')

ax.plot3D(waypoints[0,:,0],waypoints[0,:,1],waypoints[0,:,2],'w')
plt.plot(waypoints[0,:,0],waypoints[0,:,1],'o')
#%%
T = [1]*(waypoints.shape[1]-1)
print(T)
p_list = []
trajxs = []
formation={}
for i in range(len(h)):
    print('height: ',h[i])
    planner = tgen.plan_min_snap
    trajx = planner(waypoints[i,:,0],T)
    trajy = planner(waypoints[i,:,1],T)
    trajz = planner(waypoints[i,:,2],T)
    cx = trajx.c
    cy = trajy.c
    cz = trajz.c
    traj=[]
    for leg in range(len(T)):
        cxi = np.flip(cx[8*leg:8*(leg+1)],0)
        cyi = np.flip(cy[8*leg:8*(leg+1)],0)
        czi = np.flip(cz[8*leg:8*(leg+1)],0)
        cyaw = np.array([0]*8)
        traj.append(np.hstack([T[leg],cxi,cyi,czi,cyaw]))
    formation[h[i]] = np.array(traj).tolist()
    #print(formation)
    p = np.array([
        trajx.compute_trajectory()['x'],
        trajy.compute_trajectory()['x'],
        trajz.compute_trajectory()['x']
    ])
    p_list.append(p)
    trajxs.append(trajx)
plt.figure()
plt.grid()
ax = plt.axes(projection='3d')
for p in p_list:
    ax.plot3D(p[0,:],p[1,:],p[2,:])

plt.figure()
for trajx in trajxs:
    trajx.plot()
#%%
import json
path = '/home/zp/catkin_ws/src/turtlesim_cleaner/src/json/'
with open('dwTest.json', 'w') as f:
    json.dump(formation, f)
for i in range(len(h)):
    with open(str(h[i])+'.json','w') as f:
        json.dump(formation[h[i]],f)
#%%
