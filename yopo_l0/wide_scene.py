"""Deterministic tiled obstacles covering every map tile, including behind launch.

The small centre clearing is a launch safety exception. No route or destination
corridor is cleared. The card stores the generator inputs, not thousands of rows.
"""
import math
import numpy as np


def generate_wide_geometry(config):
    extent = float(config.get('half_extent_m', 1100.0))
    tile = float(config.get('tile_m', 40.0))
    clearing = float(config.get('launch_clear_radius_m', 18.0))
    launch_exception=bool(config.get('allow_empty_launch_tiles',False))
    if not (100.0 <= extent <= 10000.0 and 8.0 <= tile <= 100.0 and 0.0 <= clearing < (extent/2 if launch_exception else tile)):
        raise ValueError('invalid wide field dimensions')
    n = int(round(2.0 * extent / tile))
    if not math.isclose(n * tile, 2.0 * extent):
        raise ValueError('field size must be divisible by tile size')
    rng = np.random.default_rng(int(config.get('seed', 19)))
    trees=[]
    for ix in range(n):
        for iy in range(n):
            x0, y0 = -extent + ix*tile, -extent + iy*tile
            # One obstacle per tile; resample inside centre tiles to retain coverage.
            for attempt in range(100):
                xy=np.array([x0, y0])+rng.uniform(.15,.85,2)*tile
                radius=float(rng.uniform(.4, .85))
                if np.linalg.norm(xy) > clearing + radius:
                    break
            else:
                if launch_exception:
                    continue
                raise ValueError('launch clearing consumed a full tile')
            trees.append(dict(name=f'wide_tree_{ix}_{iy}',center_xy=xy.tolist(),radius=radius,
                              z_lo=0.0,z_hi=float(rng.uniform(5.0,9.0))))
    return {'boxes': [], 'cylinders': trees}
