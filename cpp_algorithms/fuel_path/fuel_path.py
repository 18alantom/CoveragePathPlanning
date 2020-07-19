
from .dist_matrix import dist_fill
from .fuel_path_helpers import get_refuel_idx, path_to_fuel

def get_fuel_paths(path, area_map, fuel_points, fuel_capacity):
    """
    Calculates detour paths to the closest fuel point with optimal
    fuel consumption.

    PARAMETERS
    ---
    path : the drone coverage path as a list of tuples
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
    fuel_dist_map : a matrix of distance to the closest fuel point
    detour_indices : indices of the path list where detour for 
        fuelling has to be taken
    paths_to_refuel : list of shortest path starting at `path[detour_inces[i]]` 
        and ending at `fuel_points[i]`
	"""
    fuel_dist_map = dist_fill(area_map, fuel_points)
    detour_indices = get_refuel_idx(fuel_dist_map, path, fuel_capacity)

    paths_to_refuel = []
    for idx in detour_indices:
        detour_location = path[idx]
        path_to_refuel = path_to_fuel(fuel_dist_map, detour_location)
        paths_to_refuel.append(path_to_refuel)
    
    return (
        fuel_dist_map,
        detour_indices,
        paths_to_refuel
    )

def get_spliced_paths(path, area_map, fuel_points, fuel_capacity):
    """
    Inserts detour path to the fuelling station into the original path
    and returns it.

    PARAMETERS
    ---
    path : the drone coverage path as a list of tuples
        [(x1,y1),...,(xm,ym)]

    area_map : 2 dim numpy array of the area, 
        obstacles : -1, area to map: 0

    fuel_points : non obstacle points on the map which are
        fuel_points as a list of tuples.
        [(x1,y1),...,(xm,ym)]

    fuel_capacity : int, fuel capacity of the drone interms of 
        1 = time required to move to an adjacent cell.
    """
    # TODO: Complete this function (if required?)
    raise NotImplementedError

    
