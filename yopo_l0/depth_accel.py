"""Explicit optional OpenCV NS depth preparation and D435 dual-mask helpers."""
import os
import numpy as np
from yopo_swarm_core.depth import network_depth as core_network_depth


def validate_depth_backend():
    backend=os.environ.get('YOPO_DEPTH_BACKEND','core')
    if backend not in ('core','opencv'):
        raise ValueError('YOPO_DEPTH_BACKEND must be core or opencv')
    if backend=='opencv':
        import cv2  # fail before arming if the explicit optional runtime is absent
    return backend


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


def conservative_foreground_downsample(
    raw_m: np.ndarray,
    target_hw: tuple[int, int] = (96, 160),
    *,
    min_valid_m: float = 0.2,
    max_valid_m: float = 20.0,
) -> np.ndarray:
    """Downsample high-res depth preserving thin foreground obstacles via min-pooling.

    - Any finite pixel in [min_valid_m, max_valid_m] dominates its bin by minimum depth.
    - If a bin has no finite surface return but contains unknown/zero/NaN, the bin is NaN.
    - Only bins where all rays are verified far-free (+inf) remain +inf.
    """
    src = np.asarray(raw_m, dtype=np.float32)
    if src.ndim != 2 or min(src.shape) == 0:
        raise ValueError("raw_m must be a nonempty 2D array")
    th, tw = int(target_hw[0]), int(target_hw[1])
    sh, sw = src.shape
    out = np.full((th, tw), np.nan, dtype=np.float32)
    row_edges = np.linspace(0, sh, th + 1, dtype=int)
    col_edges = np.linspace(0, sw, tw + 1, dtype=int)
    for r in range(th):
        r0, r1 = row_edges[r], max(row_edges[r] + 1, row_edges[r + 1])
        for c in range(tw):
            c0, c1 = col_edges[c], max(col_edges[c] + 1, col_edges[c + 1])
            patch = src[r0:r1, c0:c1]
            valid = np.isfinite(patch) & (patch >= min_valid_m) & (patch <= max_valid_m)
            if np.any(valid):
                out[r, c] = np.min(patch[valid])
            elif np.all(np.isposinf(patch)):
                out[r, c] = np.inf
    return out


def prepare_d435_dual_depth(
    depth_96x160_m: np.ndarray,
    *,
    valid_col_span: tuple[int, int] | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Produce (net_depth_tensor, safety_depth_m) with separate border & hole semantics.

    - Network path replicates the outermost valid columns into narrower-FOV borders
      so YOPO outer primitives are not blocked by a 0 m wall, then applies standard
      network_depth normalization/inpainting.
    - Safety path forces narrower-FOV borders and invalid holes to NaN (UnknownVoid)
      while keeping verified far-free rays as +inf.
    """
    d = np.asarray(depth_96x160_m, dtype=np.float32)
    if d.shape != (96, 160):
        raise ValueError(f"expected (96, 160) depth, got {d.shape}")
    safety = d.copy()
    safety[(~np.isnan(safety)) & (~np.isposinf(safety)) & (safety <= 0.04)] = np.nan
    net_src = d.copy()
    net_src[np.isposinf(net_src)] = np.float32(20.0)
    if valid_col_span is not None:
        c0, c1 = int(valid_col_span[0]), int(valid_col_span[1])
        if not (0 <= c0 < c1 <= 160):
            raise ValueError(f"invalid valid_col_span {valid_col_span}")
        if c0 > 0:
            safety[:, :c0] = np.nan
            net_src[:, :c0] = net_src[:, c0 : c0 + 1]
        if c1 < 160:
            safety[:, c1:] = np.nan
            net_src[:, c1:] = net_src[:, c1 - 1 : c1]
    return network_depth(net_src), safety
