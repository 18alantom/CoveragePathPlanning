# import is_valid
from cpp_algorithms.common_helpers import is_valid
from cpp_algorithms.coverage_path.pathing_helpers import get_shortest_l1_path
import numpy as np

def update_keeper(keeper, next_point):
    """
    Update values of the keeper dict.
    """
    keeper["prev_point"] = keeper["curr_point"]
    keeper["curr_point"] = next_point
    keeper["coverage_path"].append(next_point)
    
    # If everything works this conditional 
    # should evaluate to True always
    if not keeper["is_visited"][next_point]:
        keeper["is_visited"][next_point] = True
        return True
    return False
    
def get_next_valid(c_point, keeper, obstacle):
    """
    Checks points (RDLU) for one with max dist
    and not yet visited.
    """
    rdlu = lambda p:np.array([(p[0],p[1]+1),(p[0]+1,p[1]),\
                              (p[0],p[1]-1),(p[0]-1,p[1])])
    max_ = -1
    next_point = None
    for possible_point in rdlu(c_point):
        possible_point = tuple(possible_point)
        """
        is_valid = True if the point is within bounds and 
        hasn't been visited previously.
        """
        if is_valid(possible_point, keeper["is_visited"], obstacle=True):
            d_val = keeper["dist_map"][possible_point]
            if d_val > max_:
                max_ = d_val
                next_point = possible_point
    return next_point 

def backtrack_subroutine(keeper, obstacle):
    """
    Go back the coverage path, keep adding to the backtrack
    path, once a viable point is reached, stop, record backtrack
    path and backtrack start.
    """
    # Should be True : backtrack_start == backtrack_path[0].
    backtrack_start = len(keeper["coverage_path"]) - 1 # last index of coverage_path
    covered_path = keeper["coverage_path"][::-1] # reversed coverage_path
    backtrack_path = []
    
    # Covered points are obstacles.
    covered_map = keeper["dist_map"].copy()
    x,y = np.array(covered_path).T
    covered_map[x,y] = obstacle
    
    for bt_point in covered_path:
        backtrack_path.append(bt_point)
        next_point = get_next_valid(bt_point, keeper, obstacle)
        if next_point is not None:
            """
            keeper["backtrack_paths"] stores all `backtrack_path` lists
            keeper["backtrack_starts"] stores indices where the backtracking
                starts in the main keeper["coverage_path"] list.
            all elements in a `backtrack_path` are present in the `coverage_path`.
            """
            keeper["backtrack_paths"].append(backtrack_path)
            keeper["backtrack_starts"].append(backtrack_start)
            return next_point
    else:
        raise Exception("backtrack failed", covered_path[::-1])

def get_replacement_paths_l1(backtrack_paths, area_map, obstacle=-1):
    """
    Calculate the shortest l1 paths from the 
    generated backtrack paths of the wavefront algorithm.
    """
    replacement_paths = []
    for backtrack_path in backtrack_paths:
        start_point = backtrack_path[0]
        end_point = backtrack_path[-1]
        replacement_paths.append(get_shortest_l1_path(start_point, end_point, area_map, obstacle))
    return replacement_paths
