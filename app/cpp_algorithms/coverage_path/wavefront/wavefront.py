from .wavefront_caller import wavefront_caller
from .wavefront_helpers import get_replacement_paths_l1
from cpp_algorithms.common_helpers import get_random_coords
from cpp_algorithms.coverage_path.pathing_helpers import splice_paths, has_isolated_areas

def wavefront(area_map,  start_point, center_point=None, isolated_area_check=True,
              use_replacements_paths=True, splice_backtrack_paths=True):
    """
    Runs the wavefront algorithm and returns a generated path.
    
    PARAMETERS
    ---
    area_map : Area map to be covered, 2-dim numpy array. 
        coverage region =  0
        obstacle region = -1

    start_point : Drone deployment point on the `area_map`, tuple (x,y)
        
    isolated_area_check : if True will check region for isolated areas, 
        algorithm will return None.
    
    use_replacement_paths : if True will compute the shortest bactrack paths.
    
    splice_backtrack_paths : if True will splice in the backtrack paths into the
        main coverage path, if False will return a dict containing the paths
        
    
    RETURNS
    ---
    if splice_backtrack_paths=True:
        will return the entire path in the form [(x1,y1),...,(xm,ym)]
    else:
        will return a dict with:
            coverage_path : main coverage path contains no backtracks, will be 
                disconnected if there are backtracks.
                
            backtrack_paths : paths used when the algorithm needs to unstuck
                itself to get to a new region.
                
            backtrack_starts : indices in the coverage path from where the 
                backtrack paths start.
    """
    if center_point is None:
        center_point = get_random_coords(area_map, 1)[0]
    
    if isolated_area_check and has_isolated_areas(area_map):
        raise ValueError("map has isolated areas")
    
    coverage_path, backtrack_paths, backtrack_starts = wavefront_caller(area_map, start_point, center_point)
    
    if use_replacements_paths:
        backtrack_paths = get_replacement_paths_l1(backtrack_paths, area_map)
    
    if splice_backtrack_paths:
        return splice_paths(coverage_path, backtrack_starts, backtrack_paths)
    
    else:
        return {
            "coverage_path" : coverage_path,
            "backtrack_paths": backtrack_paths,
            "backtrack_starts": backtrack_starts
        }