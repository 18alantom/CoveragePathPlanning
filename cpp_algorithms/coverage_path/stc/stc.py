from .stc_helpers import stc_preprocess
from .stc_caller import stc_caller

def stc(area_map, start_point):
    """
    Runs spanning tree coverage to obtain a path
    for the given area_map.
    """
    matrix = stc_preprocess(area_map)
    coverage_paths = stc_caller(matrix)
    # full_path = path_stitching(coverage_paths)
    return coverage_paths
