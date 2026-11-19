"""Per-aircraft observed depth surfaces. No world/scene geometry is accepted."""
import numpy as np
from scipy.spatial import cKDTree


_SURFACE_VOXEL_M = 0.12


class DepthLocalField:
    def __init__(
        self,
        mount,
        max_range=20.0,
        *,
        strict_unknown: bool = False,
        swept_radius_m: float | None = None,
        cam_rotation=None,
        history_frames: int = 8,
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
        self.history_frames = history_frames
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
        self.frames = [f for f in self.frames if frame.stamp - f[0] < 2.0][-self.history_frames:]
        self.surfaces.append(points)
        self.surfaces = self.surfaces[-self.history_frames:]
        # Retain recently observed surfaces when they move behind the camera.
        cloud = np.concatenate(self.surfaces)
        if len(cloud):
            cells = np.floor(cloud / _SURFACE_VOXEL_M).astype(np.int64)
            # Collision-free mixed-radix voxel key within the supported +/-10 km worlds.
            keys = cells[:, 0] + cells[:, 1] * 1_000_000 + cells[:, 2] * 1_000_000_000_000
            _, ids = np.unique(keys, return_index=True)
            self.tree = cKDTree(cloud[ids])
        else:
            self.tree = None

    def intersects_observed_surface(self, centers, radii) -> np.ndarray:
        """Conservatively expand for displacement inside each retained surface voxel."""
        if self.tree is None:
            return np.zeros(len(centers), dtype=bool)
        distance = self.tree.query(centers, workers=1)[0]
        return distance <= radii + np.sqrt(3.0) * _SURFACE_VOXEL_M

    def measured_sphere_coverage(
        self, centers: np.ndarray, radii: np.ndarray, *, min_range_m: float, since_s: float,
    ) -> np.ndarray:
        """Return the supporting exposure for each sphere, or NaN when unobserved.

        Project each full sphere into the calibrated image. Every pixel in the
        conservative projection rectangle must establish free space past its farthest point.
        The aircraft's previously occupied body neighbourhood does not enter this test.
        """
        evidence = np.full(len(centers), np.nan)
        for stamp, origin, R_wc, K, depth, _ in reversed(self.frames):
            if stamp < since_s:
                break
            unseen = np.flatnonzero(np.isnan(evidence))
            if unseen.size == 0:
                break
            local = (centers[unseen] - origin) @ R_wc
            radius = radii[unseen]
            fx, fy, cx, cy = K
            x, y, z = local.T
            depth_ok = (x - radius > min_range_m) & (x + radius < self.max_range - 1.0)
            for i in np.flatnonzero(depth_ok):
                # Tangent-ray extrema of y/x and z/x on the sphere.
                denom = x[i] ** 2 - radius[i] ** 2
                y_span = radius[i] * np.sqrt(denom + y[i] ** 2)
                z_span = radius[i] * np.sqrt(denom + z[i] ** 2)
                u0 = int(np.floor(cx - fx * (x[i] * y[i] + y_span) / denom))
                u1 = int(np.ceil(cx - fx * (x[i] * y[i] - y_span) / denom))
                v0 = int(np.floor(cy - fy * (x[i] * z[i] + z_span) / denom))
                v1 = int(np.ceil(cy - fy * (x[i] * z[i] - z_span) / denom))
                if not (2 <= u0 <= u1 < depth.shape[1] - 2 and 2 <= v0 <= v1 < depth.shape[0] - 2):
                    continue
                if np.min(depth[v0:v1 + 1, u0:u1 + 1]) > x[i] + radius[i] + 0.6:
                    evidence[unseen[i]] = stamp
        return evidence

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
