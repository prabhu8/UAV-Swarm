#%%
import numpy as np
import matplotlib.pyplot as plt
import bswarm
import bswarm.trajectory_generation as tgen 

#%%
n_turtle = 10
r = 1
pi = np.pi

#read wayponts
inpath = 'waypoints/'
initPos = np.genfromtxt(inpath+'init_pos.csv',delimiter=',')
formC = np.genfromtxt(inpath+'C.csv',delimiter=',')
formD = np.genfromtxt(inpath+'D.csv',delimiter=',')

#%%
#initialize waypoints
waypoints = [initPos]
waypoints.append(formC)
waypoints.append(formD)
waypoints = np.array(waypoints)

print(np.shape(waypoints))

#%%
T = [10]*2
p_list = []
trajxs = []
formation={}
for turtle in range(n_turtle):
    print('turtle: ',turtle)
    planner = tgen.plan_min_accel
    trajx = planner(waypoints[:,turtle,0],T)
    trajy = planner(waypoints[:,turtle,1],T)
    cx = trajx.c
    cy = trajy.c

    traj=[]
    for leg in range(len(T)):
        cxi = cx[4*leg:4*(leg+1)]
        cyi = cy[4*leg:4*(leg+1)]
        traj.append(np.hstack([T[leg],cxi,cyi]))
    formation[turtle] = np.array(traj).tolist()
    p = np.array([trajx.compute_trajectory()['x'],
        trajy.compute_trajectory()['x']
    ])
    p_list.append(p)
    trajxs.append(trajx)
formation
plt.figure()
plt.grid()
for p in p_list:
    plt.plot(p[0,:],p[1,:])
plt.plot(initPos[:,0],initPos[:,1],'ro')
plt.plot(formC[:,0],formC[:,1],'bx')
plt.plot(formD[:,0],formD[:,1],'yx')
plt.figure()
for trajx in trajxs:
    trajx.plot()
#%%
import json
path = '/home/zp/catkin_ws/src/turtlesim_cleaner/src/json/'
with open('c2d.json', 'w') as f:
    json.dump(formation, f)



#%%
