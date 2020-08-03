from .bcd_helper import backtracking_list, move_like_oxen, bcd_preprocess
from .astar import astar_path, astar_search

def bcd_main(matrix, start_point):
    width, height = matrix.shape
    # May add support for variable radius.
    radius = 0.5
    diameter = int(2*radius)
    x, y = start_point
    memory = []
    backtrack_counts = 0
    point_find_failed = 0
    while True:
        critical_x, critical_y = move_like_oxen(matrix, diameter, x, y, memory)
        next_, is_end = backtracking_list(
            memory, diameter, matrix, critical_x, critical_y)
        x, y = next_
        if is_end:
            break
        else:
            start = (critical_x, critical_y)
            end = (x, y)
            path = astar_search(start, end, matrix, diameter, width, height)
            astar_path(matrix, memory, path)
    return memory

def bcd(area_map, start_point):
    """
    Runs the boustrophedon cell decomposition algorithm 
    with astar for point search and returns th generated path.

    PARAMETERS
    ---
    area_map : Area map to be covered, 2-dim numpy array. 
        coverage region =  0
        obstacle region = -1

    start_point : Drone deployment point on the `area_map`, tuple (x,y)

    RETURNS
    ---
    coverage_path : main coverage path.
    """
    matrix = bcd_preprocess(area_map)
    coverage_path = bcd_main(matrix, start_point)
    return coverage_path
