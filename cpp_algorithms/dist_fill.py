import numpy as np
from .constants import OB
from .common_helpers import is_valid
# from cpp_algorithms.testers.helpers import is_valid

def is_valid_vectorized(coords, v_map, obstacle=True):
    # Bound check
    assert coords.shape[1] == 2
    h,w = v_map.shape
    x,y = coords.T
    is_within_bounds = (x >= 0) & (x < h) & (y >= 0) & (y < w)
    x = np.clip(x.copy(), 0, h-1)
    y = np.clip(y.copy(), 0, w-1)
    is_not_on_obstacle = (v_map[x,y] != obstacle)
    return is_within_bounds & is_not_on_obstacle

def get_udlr(coords):
    udlr = np.array([[-1,0],[1,0],[0,-1],[0,1]])[None,:,:]
    return (coords[:,None,:] + udlr).reshape(-1,2)

def dist_fill_single(area_map, fill_point):
    v_map = area_map == OB
    dist_map = area_map.copy().astype(np.int32)
    
    assert is_valid(fill_point, area_map), \
    "invalid fill point"
    
    fval = 0
    dist_map[fill_point] = fval
    v_map[fill_point] = True
    
    to_visit = np.array([fill_point])
    while len(to_visit) > 0:
        x,y = np.array(to_visit).T
        dist_map[x,y] = fval
        v_map[x,y] = True
        udlr = np.unique(get_udlr(to_visit),axis=0)
        mask = is_valid_vectorized(udlr, v_map)
        to_visit = udlr[mask]
        fval += 1
    return dist_map

def dist_fill(area_map, fill_points):
    """
    Returns the distance matrix.

    PARAMETERS
    ---
    area_map : binary map, -1=obstacle, 0=area to be mapped.
    points : [(x,y)] points around which to dist fill (fuel station).
    """
    dist_maps = []
    for fill_point in fill_points:
        dist_maps.append(dist_fill_single(area_map, fill_point))
    return np.array(dist_maps).min(axis=0)