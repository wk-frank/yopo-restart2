"""Visualize six archived Gazebo float-depth frames from the completed flight."""
from pathlib import Path
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

dest=Path(sys.argv[1])
fig,axes=plt.subplots(2,3,figsize=(12.8,7.2),facecolor='#f6f8fb')
cmap=plt.get_cmap('turbo_r').copy();cmap.set_bad('#e9edf2')
for i,ax in enumerate(axes.flat,1):
    files=sorted((dest/'depth-samples').glob(f'px4_{i}-*.npz'))
    assert files
    sample=np.load(files[len(files)//2])
    im=ax.imshow(sample['depth'],vmin=.2,vmax=20,cmap=cmap,interpolation='nearest')
    ax.set_title(f"UAV {i-1} | Gazebo {float(sample['gazebo_stamp_s']):.3f} s",fontsize=12)
    ax.set(xlabel='Pixel u',ylabel='Pixel v')
fig.suptitle('Six Gazebo native depth inputs | R_FLOAT32 / metres',fontsize=18,y=.98)
fig.subplots_adjust(left=.05,right=.90,top=.9,bottom=.1,hspace=.32,wspace=.25)
bar=fig.add_axes([.925,.18,.018,.64]);fig.colorbar(im,cax=bar,label='Optical-axis depth (m)')
fig.text(.5,.025,'Actual archived sensor data; colours for display only. Grey = invalid / no return. No scene-card rendering.',ha='center',fontsize=10,color='#526171')
fig.savefig(dest/'native-depth-six.png',dpi=130)
print('Six-camera native-depth panel written')
