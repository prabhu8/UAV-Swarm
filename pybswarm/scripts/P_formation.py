#%%
import bswarm
import bswarm.formation as form
import bswarm.trajectory_generation as tgen

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

%matplotlib inline
#%%

#init pos
P = np.array([
    [-1, 1, 1],
    [0, 1, 1],
    [1, 1, 1],
    [-1, 0, 1],
    [0, 0, 1],
    [1, 0, 1],
    [-1, -1, 1]
]).T
ax = plt.axes(projection='3d')
ax.plot3D(P[0,:], P[1,:], P[2,:], 'ro')

#%%

#target pos
P2 = np.array([
    [-1, 1, 2.5],
    [0, 1, 2.5],
    [1, 1, 2.5],
    [-1, 0, 1.75],
    [0, 0, 1.75],
    [1, 0, 1.75],
    [-1, -1, 1]
]).T
ax = plt.axes(projection='3d')
ax.plot3D(P[0,:], P[1,:], P[2,:], 'ro')
ax.plot3D(P2[0,:], P2[1,:], P2[2,:], 'bo')

#%%

waypoints = [P]
for theta in np.linspace(0, 2*np.pi, 8):
    waypoints.append(form.rotate_points_z(P2, theta))
waypoints.append(P)
waypoints = np.array(waypoints)
print(np.shape(waypoints))
ax = plt.axes(projection='3d')
for point in range(waypoints.shape[2]):
    ax.plot3D(waypoints[:,0,point], waypoints[:,1,point], waypoints[:,2,point], '-')
    ax.view_init(azim=0, elev=40)

#%%

dist = np.linalg.norm(waypoints[1:, :, :] - waypoints[:-1, :, :], axis=1)
dist_max = np.max(dist, axis=1)
dist_max

ax = plt.axes(projection='3d')
p_list = []
trajx_list = []
T = 1*np.ones(dist.shape[0])
print('T', T)

formation = {}

for drone in range(waypoints.shape[2]):
    print('drone', drone)
    planner = tgen.plan_min_snap
    trajx = planner(waypoints[:, 0, drone], T)
    trajy = planner(waypoints[:, 1, drone], T)
    trajz = planner(waypoints[:, 2, drone], T)
    
    cx = trajx.c
    cy = trajy.c
    cz = trajz.c
    cyaw = np.array([0]*8)
    traj = []
    formation[drone] = []
    for leg in range(len(T)):
        cxi = np.flip(cx[8*leg:8*(leg+1)],0)
        cyi = np.flip(cy[8*leg:8*(leg+1)],0)
        czi = np.flip(cz[8*leg:8*(leg+1)],0)
        traj.append(np.hstack([T[leg],cxi,cyi,czi,cyaw]))
    formation[drone] = np.array(traj).tolist()
    print('traj',formation[drone])
    
    p = np.array([trajx.compute_trajectory()['x'],
        trajy.compute_trajectory()['x'],
        trajz.compute_trajectory()['x']])
    p_list.append(p)
    trajx_list.append(trajx)

    header = 'Duration,x^0,x^1,x^2,x^3,x^4,x^5,x^6,x^7,y^0,y^1,y^2,y^3,y^4,y^5,y^6,y^7,z^0,z^1,z^2,z^3,z^4,z^5,z^6,z^7,yaw^0,yaw^1,yaw^2,yaw^3,yaw^4,yaw^5,yaw^6,yaw^7'
    np.savetxt(repr(drone)+'.csv',np.array(formation[drone]),delimiter=',',header=header)

plt.figure()
for p in p_list:
    ax.plot3D(p[0,:], p[1,:], p[2,:])

plt.figure()
for trajx in trajx_list:
    trajx.plot()
#%%
import json

path = 'trajectory/json/'
with open('formP.json', 'w') as f:
    json.dump(formation, f)

#%%


#%%


#%%
