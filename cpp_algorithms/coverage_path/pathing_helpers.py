import numpy as np
from cpp_algorithms.common_helpers import is_valid, get_random_coords

get_adj = lambda x,y :[
    (x+1,y),(x-1,y),(x,y+1),(x,y-1)
]

get_adj_8 = lambda x,y :[
    (x+1,y),(x-1,y),(x,y+1),(x,y-1),
    (x+1,y+1),(x-1,y-1),(x-1,y+1),(x+1,y-1)
]

def splice_paths(coverage_path, splice_indices, splice_segments):
    """
    Splices in `splice_segments` at the given `splice_indices` for
    the given `coverage_path`
    """
    assert len(splice_indices) == len(splice_segments), "length discrepancy"
    full_path_segments = []
    last_idx = 0
    for i,idx in enumerate(splice_indices):
        seg = np.array(coverage_path[last_idx:idx])
        if len(seg) > 0:
            full_path_segments.append(seg)
        seg = np.array(splice_segments[i])
        if len(seg) > 0:
            full_path_segments.append(seg)
        last_idx = idx + 1
        
    full_path_segments.append(np.array(coverage_path[last_idx:]))
    return np.concatenate(full_path_segments)

def has_isolated_areas(area_map, obstacle=-1):
    """
    Flood fills the area to check if there are 
    isolated areas.
    """
    
    v_map = (area_map == obstacle).copy()
    f_point = get_random_coords(area_map, 1, obstacle)[0]
    to_visit = [f_point]
    
    while len(to_visit) > 0:
        l = len(to_visit)
        for point in to_visit:
            v_map[point] = True
            
        for i in range(l):
            for adj_point in get_adj(*to_visit.pop(0)):
                if is_valid(adj_point, area_map, obstacle) \
                    and not v_map[adj_point] \
                    and adj_point not in to_visit:
                    to_visit.append(adj_point)
                    
    if v_map.sum() == v_map.size:
        return False
    return True

def get_step(path_map, next_point, obstacle=-1):
    min_d_val = path_map[next_point]
    possible_point = None
    for adj_point in get_adj(*next_point):
        if is_valid(adj_point, path_map, obstacle):
            d_val = path_map[adj_point]
            if d_val < min_d_val:
                min_d_val = d_val
                possible_point = adj_point
    return possible_point, min_d_val

def get_path(path_map, start_point, end_point, obstacle=-1):
    """
    Get the shortest (directed) l1 distance
    between `start_point`, `end_point` for a given `path_map`.
    
    (directed ∵ distance should be min at end_point)
    """
    path = [start_point]
    end_val = path_map[end_point]
    cell = path_map[start_point]
    next_point = start_point
    while cell != end_val:
        next_point, cell = get_step(path_map, next_point, obstacle)
        path.append(next_point)
    return path

def wave_find_map(start_point, end_point, area_map, obstacle=-1):
    """
    Creates a path map, which is a dist map that terminates once
    the required point is found.
    """
    AREA_VAL = 0
    d_map = np.int64(area_map.copy())
    v_map = (area_map == obstacle).copy()
    to_visit = [end_point]
    def loop():
        d_val = 1
        while len(to_visit) > 0:
            l = len(to_visit)
            for point in to_visit:
                d_map[point] = d_val
                v_map[point] = True
                if point == start_point:
                    return
                # check if point is the start or something

            d_val += 1

            for i in range(l):
                for adj_point in get_adj(*to_visit.pop(0)):
                    if is_valid(adj_point, area_map, obstacle) \
                        and not v_map[adj_point] \
                        and adj_point not in to_visit:
                        to_visit.append(adj_point)
                        
    loop()
    d_map[d_map == AREA_VAL] = obstacle
    return d_map

def get_shortest_l1_path(start_point, end_point, area_map, obstacle=-1):
    """
    Returns the shortest Manhattan path between the two points
    taking obstacles into consideration.
    """
    path_map = wave_find_map(start_point, end_point, area_map, obstacle)
    return get_path(path_map, start_point, end_point, obstacle)