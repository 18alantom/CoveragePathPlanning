import numpy as np
from .constants import FU, OB

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
    path = [loc]
    cell = dist_map[loc]
    while cell != fuel:
        loc, cell = get_step(dist_map, loc, obs)
        path.append(loc)
    return path

def get_refuel_idx(dist_map, path, fuel_cap):
    """
    PARAMETERS
    ---
    dist_map : distance map where fuel points are 0 valued, 
        (m,n) shape nd array
    path : sequence of coords that traverses the `dist_map`.
        list of tuples, [(x1,y1),...,(xm,ym)]
    fuel_cap : capacity of the fuel required by the drone, int.
    
    RETURNS
    ---
    points : indices in the path where refuelling detour is taken.
    fcap : fuel capacity at every point in the path.
    """
    path = np.array(path)
    x,y = path.T
    dist = dist_map.copy()[x,y]
    plen = dist.shape[0]
    fcap = np.full(dist.shape, 0)

    points = []

    cu = 0 # path distance travelled by the drone
    fuel_req = 0 # initial req 0 cause starting with full tank

    while True:
        ini_fuel = (fuel_cap - fuel_req)

        assert ini_fuel > 0, "no fuel left to complete path" 

        """
        Start index : how many points have previously been covered
        End index   : how many points can be covered with available fuel
        """
        start = cu
        end = cu + ini_fuel
        
        """
        If end index overshoots path length,
        end index should be the path length (ie last position -1)
        """
        if plen - end < 0:
            end = plen
        sl = slice(start, end)

        """
        Amount of distance that can be travelled by the drone
        taking into account the last loc.
        """
        p_seg_len = end - start
        fin_fuel = ini_fuel - p_seg_len

        """
        Possible fuel remaining starting index 
        """
        fcap[sl] = np.arange(ini_fuel, fin_fuel, -1)
        """
        if : Remaining fuel is sufficient.
        """
        if ini_fuel - (plen - cu) >=0:
            break

        freq = fcap[sl] - dist[sl]
        try:
            idx = freq[freq >= 0].argmin()
        except ValueError:
            raise ValueError("path can't be completed, insufficient fuel capacity")

        cu += idx

        points.append(cu)
        fuel_req = dist[cu]
    return points, fcap