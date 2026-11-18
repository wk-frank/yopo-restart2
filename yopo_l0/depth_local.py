"""Per-aircraft observed depth surfaces. No world/scene geometry is accepted."""
import numpy as np
from scipy.spatial import cKDTree


class DepthLocalField:
    def __init__(
        self,
        mount,
        max_range=20.0,
        *,
        strict_unknown: bool = False,
        swept_radius_m: float | None = None,
        cam_rotation=None,
    ):
        self.mount = np.asarray(mount, dtype=float)
        self.max_range = float(max_range)
        self.strict_unknown = bool(strict_unknown)
        self.swept_radius_m = (
            (0.35 if self.strict_unknown else 1.2)
            if swept_radius_m is None
            else float(swept_radius_m)
        )
        self.cam_rotation = (
            np.eye(3, dtype=float)
            if cam_rotation is None
            else np.asarray(cam_rotation, dtype=float).reshape(3, 3)
        )
        self.has_current_evidence = False
        self.frames = []
        self.tree = None
        self.stamp = None
        self.surfaces = []

    def observe(self, frame, position, rotation):
        if self.stamp is not None and abs(self.stamp - frame.stamp) < 1e-6:
            return
        self.stamp = frame.stamp
        p = np.asarray(position, dtype=float)
        R_wb = np.asarray(rotation, dtype=float)
        R_wc = R_wb @ self.cam_rotation
        origin = p + R_wb @ self.mount
        d = np.asarray(frame.depth)
        fx, fy, cx, cy = frame.K
        v, u = np.indices(d.shape)
        valid = np.isfinite(d) & (d > 0.2) & (d < self.max_range)
        # Camera +X forward, +Y left, +Z up; pixels right/down.
        rays = np.stack([np.ones_like(u), -(u - cx) / fx, -(v - cy) / fy], axis=-1)
        points = (rays[valid] * d[valid, None]) @ R_wc.T + origin
        self.has_current_evidence = bool(valid.any() or np.isposinf(d).any())
        swept_center = p.copy() if self.strict_unknown else origin
        self.frames.append((frame.stamp, origin, R_wc, frame.K, d, swept_center))
        self.frames = [f for f in self.frames if frame.stamp - f[0] < 2.0][-8:]
        self.surfaces.append(points)
        self.surfaces = self.surfaces[-8:]
        # Retain recently observed surfaces when they move behind the camera.
        cloud = np.concatenate(self.surfaces)
        if len(cloud):
            cells = np.floor(cloud / 0.12).astype(np.int64)
            # Collision-free mixed-radix voxel key within the supported +/-10 km worlds.
            keys = cells[:, 0] + cells[:, 1] * 1_000_000 + cells[:, 2] * 1_000_000_000_000
            _, ids = np.unique(keys, return_index=True)
            self.tree = cKDTree(cloud[ids])
        else:
            self.tree = None

    def min_distance(self, pts):
        pts = np.asarray(pts).reshape(-1, 3)
        if not self.frames or (self.strict_unknown and not self.has_current_evidence):
            return 0.0
        known = np.zeros(len(pts), dtype=bool)
        for _, origin, R_wc, K, depth, swept_center in self.frames:
            local = (pts - origin) @ R_wc
            x = local[:, 0]
            fx, fy, cx, cy = K
            u = np.rint(cx - fx * local[:, 1] / np.maximum(x, 0.001)).astype(int)
            v = np.rint(cy - fy * local[:, 2] / np.maximum(x, 0.001)).astype(int)
            visible = (
                (x > 0.2)
                & (x < self.max_range - 1)
                & (u >= 2)
                & (u < depth.shape[1] - 2)
                & (v >= 2)
                & (v < depth.shape[0] - 2)
            )
            ids = np.where(visible)[0]
            if len(ids):
                measured = depth[v[ids], u[ids]]
                if self.strict_unknown or np.isposinf(depth).any():
                    # Tri-state mask: +inf is verified far-clip free space; NaN is unknown void.
                    far_free = np.isposinf(measured)
                    in_front = np.isfinite(measured) & (x[ids] < measured - 0.6)
                    if self.strict_unknown:
                        in_front &= measured < self.max_range
                    known[ids] |= far_free | in_front
                else:
                    # Legacy Gazebo renderer mode where NaN denotes no-return beyond far clip.
                    known[ids] |= (~np.isfinite(measured)) | (x[ids] < measured - 0.6)
            # Immediate swept neighbourhood bounded by verified body/sweep radius.
            known |= np.linalg.norm(pts - swept_center, axis=1) < self.swept_radius_m
        if not known.all():
            return 0.0
        return float(self.tree.query(pts, workers=1)[0].min()) if self.tree is not None else self.max_range
