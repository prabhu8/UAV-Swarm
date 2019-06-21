#%%
import numpy as np
import matplotlib.pyplot as plt 
from mpl_toolkits import mplot3d
#%%
data = np.genfromtxt('spiral.csv', delimiter=',')
ax = plt.axes(projection='3d')
ax.plot3D(data[:,0],data[:,1],data[:,2],'w.')