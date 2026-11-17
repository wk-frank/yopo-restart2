"""Per-aircraft observed depth surfaces. No world/scene geometry is accepted."""
import numpy as np
from scipy.spatial import cKDTree


class DepthLocalField:
    def __init__(self, mount, max_range=20.):
        self.mount=np.asarray(mount,dtype=float)
        self.max_range=float(max_range)
        self.frames=[]
        self.tree=None
        self.stamp=None
        self.surfaces=[]

    def observe(self, frame, position, rotation):
        if self.stamp is not None and abs(self.stamp-frame.stamp)<1e-6:
            return
        self.stamp=frame.stamp
        p=np.asarray(position,dtype=float); R=np.asarray(rotation,dtype=float)
        origin=p+R@self.mount
        d=np.asarray(frame.depth); fx,fy,cx,cy=frame.K
        v,u=np.indices(d.shape)
        valid=np.isfinite(d)&(d>.2)&(d<self.max_range)
        # Camera +X forward, +Y left, +Z up; pixels right/down.
        rays=np.stack([np.ones_like(u),-(u-cx)/fx,-(v-cy)/fy],axis=-1)
        points=(rays[valid]*d[valid,None])@R.T+origin
        self.frames.append((frame.stamp,origin,R,frame.K,d))
        self.frames=[f for f in self.frames if frame.stamp-f[0]<2.0][-8:]
        self.surfaces.append(points)
        self.surfaces=self.surfaces[-8:]
        # Retain recently observed surfaces when they move behind the camera.
        cloud=np.concatenate(self.surfaces)
        if len(cloud):
            cells=np.floor(cloud/.12).astype(np.int64)
            # Collision-free mixed-radix voxel key within the supported +/-10 km worlds.
            keys=cells[:,0]+cells[:,1]*1_000_000+cells[:,2]*1_000_000_000_000
            _,ids=np.unique(keys,return_index=True)
            self.tree=cKDTree(cloud[ids])
        else:
            self.tree=None

    def min_distance(self, pts):
        pts=np.asarray(pts).reshape(-1,3)
        if not self.frames:
            return 0.
        known=np.zeros(len(pts),dtype=bool)
        for _,origin,R,K,depth in self.frames:
            local=(pts-origin)@R
            x=local[:,0]; fx,fy,cx,cy=K
            u=np.rint(cx-fx*local[:,1]/np.maximum(x,.001)).astype(int)
            v=np.rint(cy-fy*local[:,2]/np.maximum(x,.001)).astype(int)
            visible=(x>.2)&(x<self.max_range-1)&(u>=2)&(u<depth.shape[1]-2)&(v>=2)&(v<depth.shape[0]-2)
            ids=np.where(visible)[0]
            if len(ids):
                measured=depth[v[ids],u[ids]]
                # NaN is the renderer's no-return beyond far clip; only within the camera frustum.
                known[ids]|=(~np.isfinite(measured))|(x[ids]<measured-.6)
            # Immediate swept neighbourhood has already been occupied by this aircraft.
            known|=np.linalg.norm(pts-origin,axis=1)<1.2
        if not known.all():
            return 0.
        return float(self.tree.query(pts,workers=1)[0].min()) if self.tree is not None else self.max_range
