"""Explicit optional OpenCV NS depth preparation; frozen core/defaults unchanged."""
import os
import numpy as np
from yopo_swarm_core.depth import network_depth as core_network_depth


def network_depth(raw_m):
    backend=os.environ.get('YOPO_DEPTH_BACKEND','core')
    if backend=='core':
        return core_network_depth(raw_m)
    if backend!='opencv':
        raise ValueError('YOPO_DEPTH_BACKEND must be core or opencv')
    import cv2
    d=np.asarray(raw_m,dtype=np.float32)
    if d.ndim!=2 or min(d.shape)==0:
        raise ValueError('depth must be a nonempty 2D image')
    y=np.floor(np.arange(96)*(d.shape[0]/96)).astype(int)
    x=np.floor(np.arange(160)*(d.shape[1]/160)).astype(int)
    normalized=np.minimum(d[np.ix_(y,x)],np.float32(20))/np.float32(20)
    holes=np.isnan(normalized)|(normalized<np.float32(.04/20))
    u8=np.nan_to_num(normalized*np.float32(255),nan=0,neginf=0).astype(np.uint8)
    if np.any(holes) and not np.all(holes):
        u8=cv2.inpaint(u8,holes.astype(np.uint8),1,cv2.INPAINT_NS)
    return (u8.astype(np.float32)/np.float32(255))[None,None]
