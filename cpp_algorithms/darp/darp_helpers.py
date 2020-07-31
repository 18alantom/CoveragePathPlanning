import numpy as np
from cpp_algorithms.dist_fill import dist_fill
from cpp_algorithms.constants import OB

def rmse(a,b):
    """
    Root Mean Square Error
    """
    return ((a - b)**2).mean()**0.5

def get_obstacle_coords(area_map, obstacle=OB):
    """
    Returns coords of obstacles.
    """
    return np.stack(list(np.where(area_map == obstacle))).T

def has_obstacles(area_map, obstacle=OB):
    """
    True if the area map has obstacles
    """
    return (area_map == obstacle).sum() > 0

def get_evaluation_matrices(start_points, area_map):
    """
    Returns evaluation matrices for all the drones.
    Which are distance maps from their start_points
    """
    return np.float32(np.stack([dist_fill(area_map,[sp]) for sp in start_points]))

def get_assignment_matrix(E, obstacle=OB):
    """
    Assigns coord depending on evaluation matrices 
    and returns the matrix.
    """
    mask = (E[0] == obstacle)
    A = E.argmin(axis=0)
    A[mask] = -1
    return A

def get_assigned_count(A, arr=True, obstacle=OB):
    """
    A : assignment matrix
    return count dict
    """
    c = {}
    uc = np.unique(A, return_counts=True)
    if not arr:
        for drone,count in zip(*uc):
            if drone==obstacle:
                continue
            else:
                c[drone] = count
        return c
    else:
        return uc[1][uc[0] != OB]

def get_no_obs_count(area_map, obstacle=OB):
    """
    Return count of points that are not obstacles
    """
    return (area_map != obstacle).sum()

def get_coverage_ratio(drone_speed, drone_coverage):
    """
    Normalized values of the drone coverage per unit time
    """
    coverage_per_time = (drone_coverage**0.5)*drone_speed
    return coverage_per_time/coverage_per_time.sum()

def get_c(area_map):
    """
    area_map : for size calculation
    """
    learning_rate = np.log10(area_map.size) # learning_rate
    return 10**(-np.ceil(learning_rate))    # tunable parameter c

def get_fair_share(i, assigned_count, nobs_count, coverage_ratio):
    """
    assigned_count : number of cells assigned to each drone
    nobs_count : total number of cells that can be covered
    coverage_ratio : ratio of cells to be assigned to each drone
    """
    try:
        return assigned_count[i] - np.round(coverage_ratio[i] * nobs_count)
    except IndexError as error:
        # Note : if this error is thrown something is wrong.
        raise IndexError("the vicissitudes of cyclic coordinate descent"\
                        +"\n"+ "count of assigned areas has dropped below `n`"+"\n"+repr(error))