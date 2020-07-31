import numpy as np
from cpp_algorithms.dist_fill import dist_fill
from cpp_algorithms.common_helpers import is_valid
from .wavefront_helpers import get_next_valid, update_keeper, backtrack_subroutine

def wavefront_update(keeper, obstacle):
    c_point = keeper["curr_point"]
    is_new = False
    next_point = get_next_valid(c_point, keeper, obstacle)
    
    if next_point is not None:
        # non backtrack condition.
        is_new = update_keeper(keeper, next_point)
    else:
        # backtrack condition, no viable point found
        next_point = backtrack_subroutine(keeper, obstacle)
        is_new = update_keeper(keeper, next_point)
    
    # is_new should always be True,
    # else backtrack has failed and there is an error,
    # or the map has locked in regions.
    return is_new

def wavefront_follow(dist_map, start_point, obstacle):
    # Matrix of values visited.
    is_visited = np.full(dist_map.shape,False)
    is_visited[dist_map==-1] = True
    is_visited[start_point] = True
    
    # Number of points to cover.
    points_to_cover = dist_map.size - (dist_map == obstacle).sum() - 1
    
    keeper = {
        "is_backtracking":False,
        "curr_point": start_point,
        "prev_point": start_point,
        "coverage_path": [start_point],
        "backtrack_paths": [],
        "backtrack_starts": [],
        "is_visited": is_visited,
        "dist_map": dist_map
    }
    
    while points_to_cover > 0:
        is_new = wavefront_update(keeper, obstacle)
        if is_new:
            points_to_cover -= 1
    return keeper["coverage_path"], keeper["backtrack_paths"], keeper["backtrack_starts"]

def wavefront_caller(area_map, start_point, center_point, obstacle=-1):
    """
    The main wavefront algorithm.
    start_point, center_point : form (x,y)
    
    return :
        coverage_path : path followed on for coverage
        backtrack_paths : paths followed to get to uncovered point, 
            subsets of coverage_path.
        backtrack_starts : starting indices of the backtrack paths
    """
    assert is_valid(start_point, area_map, obstacle), "invalid start"
    assert is_valid(center_point, area_map, obstacle), "invalid center"
    
    dist_map = dist_fill(area_map, [center_point])
    return wavefront_follow(dist_map, start_point, obstacle)
