from .stc_helpers import stc_preprocess
from .stc_caller import stc_caller

def stc(area_map, start_point):
    """
    Runs spanning tree coverage to obtain a path
    for the given area_map.
    """
    matrix = stc_preprocess(area_map)
    coverage_path = stc_caller(matrix)
    idx = coverage_path.index(start_point)
    coverage_path = [*coverage_path[idx:],*coverage_path[:idx]]
    return coverage_path
