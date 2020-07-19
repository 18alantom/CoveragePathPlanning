import numpy as np
from .constants import OB

def is_valid(coord, shape):
    """
    Checks if a coord is within bounds.
    """
    x,y = coord
    g,h = shape
    lesser = x < 0 or y < 0
    greater = x >= g or y >= h
    if lesser or greater:
        return False
    return True
    
def get_udlr(dist_map, v_map, center, to_vis):
    """
    Returns valid non diagonal steps.
    """
    x,y = center
    udlr = [(x,y+1),(x,y-1),(x-1,y),(x+1,y)]
    sh = dist_map.shape
    for coord in udlr:
        if is_valid(coord, sh):
            is_nob = dist_map[coord] != OB
            is_vis = v_map[coord]
            is_pre = coord in to_vis
            if is_nob and not is_vis and not is_pre:
                to_vis.append(coord)
                
def dist_fill(area_map, points):
    """
    PARAMETERS
    ---
    area_map : binary map, -1=obstacle, 0=area to be mapped.
    points : [(x,y)] points around which to dist fill (fuel station).
    """
    shape = area_map.shape
    dist_map = []
    v_map = []
    for _ in points:
        dm = area_map.copy()
        dist_map.append(np.int64(dm)) # obs = -1
        v_map.append((dm == OB).copy())
    
    for i,point in enumerate(points):
        assert is_valid(point, area_map.shape), \
        "invalid coordinate for center"
        assert dist_map[i][point] != OB, \
        "center is an obstacle"
    
    for i in range(len(points)):
        pval = 0
        center = points[i]
        dist_map[i][center] = pval
        v_map[i][center] = True

        to_visit = []
        get_udlr(dist_map[i], v_map[i], center, to_visit)
        while len(to_visit) > 0:
            pval += 1
            l = len(to_visit)
            for coord in to_visit:
                dist_map[i][coord] = pval
                v_map[i][coord] = True

            for i_ in range(l):
                coord = to_visit.pop(0)
                get_udlr(dist_map[i], v_map[i], coord, to_visit)
    return np.array(dist_map).min(axis=0)