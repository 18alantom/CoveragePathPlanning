import numpy as np
from copy import deepcopy
from cpp_algorithms.constants import FU, OB

def udlr(x,y):
    # Up Down Left Right
    return [(x,y+1),(x,y-1),(x-1,y),(x+1,y)]

def get_step(dist_map, point, obs=-1):
    # Get smallest L1 step from point.
    sh = dist_map.shape
    min_ = float('inf')
    p = point
    for direc in udlr(*point):
        x,y = direc
        if x < 0 or y < 0 or \
            x >= sh[0] or y >= sh[1]:
            continue
        val = dist_map[direc]
        if val < min_ and val != obs:
            min_ = val
            p = direc
    return p, min_

def path_to_fuel(dist_map, loc, fuel=FU, obs=OB):
    """
    PARAMETERS
    ---
    dist_map : copy of the area_map 
    loc : (x,y) robot location
    fuel : value of fuel on the `dist_map`
    obs : value of obstacle on the `dist_map`
    """
    loc = tuple(loc)
    path = [loc]
    cell = dist_map[loc]
    while cell != fuel:
        loc, cell = get_step(dist_map, loc, obs)
        path.append(loc)
    return path

def get_refuel_idx(dist_map, coverage_path, fuel_cap):
    """
    PARAMETERS
    ---
    dist_map : distance map where fuel points are 0 valued, 
        (m,n) shape nd array
    coverage_path : sequence of coords that traverses the `dist_map`.
        list of tuples, [(x1,y1),...,(xm,ym)]
    fuel_cap : capacity of the fuel required by the drone, int.
    
    RETURNS
    ---
    points : indices in the coverage_path where refuelling detour is taken.
    fcap : fuel capacity at every point in the coverage_path.
    """
    coverage_path = np.array(coverage_path)
    x,y = coverage_path.T
    dist = dist_map.copy()[x,y]
    plen = dist.shape[0]
    fcap = np.full(dist.shape, 0)

    points = []

    cu = 0 # coverage_path distance travelled by the drone
    fuel_req = 0 # initial req 0 cause starting with full tank

    i = 0
    while True:
        i+=1
        ini_fuel = (fuel_cap - fuel_req)
        assert ini_fuel > 0, "no fuel left to complete coverage_path" 
        # start : how many points have previously been covered.
        # end   : how many points can be covered with available fuel.
        start = cu
        end = cu + ini_fuel
        
        # If end index overshoots coverage_path length,
        # end index should be the coverage_path length (ie last position -1)
        if plen - end < 0:
            end = plen
        sl = slice(start, end)

        # Amount of distance that can be travelled by the drone
        # taking into account the last loc.
        p_seg_len = end - start
        fin_fuel = ini_fuel - p_seg_len

        # Possible fuel remaining starting index 
        fcap[sl] = np.arange(ini_fuel, fin_fuel, -1)
        # if : Remaining fuel is sufficient.
        if ini_fuel - (plen - cu) >=0:
            break

        freq = fcap[sl] - dist[sl]
        try:
            idx = freq[freq >= 0].argmin()
        except ValueError:
            raise ValueError("coverage path can't be traversed, insufficient fuel capacity")
        
        if len(points) > 1 and points[-1] == points[-2]:
            raise ValueError(f"coverage path can't be traversed, insufficient fuel capacity\n stuck at coverage path index :{points[-1]}")

        cu += idx

        points.append(cu)
        fuel_req = dist[cu]
    return points, fcap

def splice_paths(coverage_path, fuel_paths, detour_idx, double_center=False):
    """
    Inserts detour path to the fuelling station into the original path
    and returns it along with start and end indices of the full path which.

    PARAMETERS
    ---
    coverage_path : the drone coverage path as a list of tuples
        [(x1,y1),...,(xm,ym)]

    fuel_paths : list of fuel paths
        [[(x1,y1),...,(xn,yn)],...,[...]]

    detour_idx : indices where the drone takes a detour from the main path.

    double_center : whether the fuel point coordinate should be repeated.


    RETURNS
    ---
    full_path : The entire path of the drone on the map.

    detour_start_end_idx : Indices of the full path where
        detour starts and ends, they are the same coords.
        (start, end) 
    """
    coverage_path = deepcopy(coverage_path)
    fuel_paths = deepcopy(fuel_paths)
    detour_start_end_idx = []
    segments = []
    last_idx = 0
    cumu_len = 0
    for idx,fuel_path in zip(detour_idx,fuel_paths):
        s_e = []
        
        seg = np.array(coverage_path[last_idx:idx])
        segments.append(seg)
        cumu_len += seg.shape[0]
        s_e.append(cumu_len)
        
        if double_center:
            splice_in = [*fuel_path,*fuel_path[::-1][:-1]]
        else:
            splice_in = [*fuel_path,*fuel_path[::-1][1:-1]]
        
        seg = np.array(splice_in)
        cumu_len += seg.shape[0]
        s_e.append(cumu_len)
        
        detour_start_end_idx.append(tuple(s_e))
        segments.append(seg)
        last_idx = idx
    
    segments.append(coverage_path[last_idx:])
    full_path = np.concatenate(segments)
    
    return full_path, detour_start_end_idx