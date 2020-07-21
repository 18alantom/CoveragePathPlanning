
import numpy as np
from copy import deepcopy

from .dist_matrix import dist_fill
from .fuel_path_helpers import get_refuel_idx, path_to_fuel

def get_fuel_paths(coverage_path, area_map, fuel_points, fuel_capacity):
    """
    Calculates detour paths to the closest fuel point with optimal
    fuel consumption.

    PARAMETERS
    ---
    coverage_path : the drone coverage path as a list of tuples
        [(x1,y1),...,(xm,ym)]

    area_map : 2 dim numpy array of the area, 
        obstacles : -1, area to map: 0

    fuel_points : non obstacle points on the map which are
        fuel_points as a list of tuples.
        [(x1,y1),...,(xm,ym)]

    fuel_capacity : int, fuel capacity of the drone interms of 
        1 = time required to move to an adjacent cell.


    RETURNS
    ---
    fuel_dist_map : a matrix of distance to the closest fuel point.

    detour_indices : indices of the path list where detour for 
        fuelling has to be taken.

    paths_to_refuel : list of shortest path starting at `path[detour_inces[i]]` 
        and ending at `fuel_points[i]`.

    fuel_capacity : list of how much fuel is remaining, same lenght as path.
	"""
    fuel_dist_map = dist_fill(area_map, fuel_points)
    detour_indices, fuel_capacity = get_refuel_idx(fuel_dist_map, coverage_path, fuel_capacity)

    paths_to_refuel = []
    for idx in detour_indices:
        detour_location = coverage_path[idx]
        path_to_refuel = path_to_fuel(fuel_dist_map, detour_location)
        paths_to_refuel.append(path_to_refuel)
    
    return (
        fuel_dist_map,
        detour_indices,
        paths_to_refuel,
        fuel_capacity
    )

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

    
