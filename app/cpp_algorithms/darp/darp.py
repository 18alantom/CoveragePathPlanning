import numpy as np
from cpp_algorithms.constants import OB
from .iterate import iterate
from .continuity import get_flood_matrix
from .darp_helpers import get_evaluation_matrices, get_c
from .darp_helpers import get_no_obs_count, get_coverage_ratio


def darp(epochs, area_map, start_points, drone_speed=None, drone_coverage=None, use_flood=True, pbar=False, obstacle=OB):
    """
    Runs DARP on the given area map and returns the assignement matrix.

    PARAMETERS
    ---
    epochs : number of iterations to iterate the loop in the iterate function for.
    area_map : area map to be divided
    start_points : start points of the drones
    drone_speed : speed of each drone in distance/time
    drone_coverage : coverage area of each drone in distance^2
    use_flood : whether to use the l1 flood fill distance, (takes time and a lot of RAM, more accurate)
    pbar : whether to show the progress bar, True – shows the pbar.

    RETURNS
    ---
    Assignment matrix
    """
    n = len(start_points)
    c = get_c(area_map)
    E = get_evaluation_matrices(start_points, area_map)

    flood_matrix = None
    # Flood matrix
    if use_flood:
        flood_matrix = get_flood_matrix(area_map, pbar=pbar)

    if drone_coverage is None or drone_speed is None:
        drone_speed = np.ones(n)
        drone_coverage = np.ones(n)

    nobs_count = get_no_obs_count(area_map)
    coverage_ratio = get_coverage_ratio(drone_speed, drone_coverage)

    return iterate(epochs, start_points, E, c, nobs_count, coverage_ratio, flood_matrix, pbar=pbar)
