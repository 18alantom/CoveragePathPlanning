from cpp_algorithms.darp import darp
from cpp_algorithms.coverage_path.bcd import bcd
from cpp_algorithms.coverage_path.stc import stc
from cpp_algorithms.common_helpers import get_drone_map


def cpp(area_map, start_points, online=False, epochs=300, use_flood=True,
        drone_speed=None, drone_coverage=None, pbar=False):
    """
    The main coverage path caller will return coverage_paths for
    each of the sub areas depending on the number of `start_points`.

    PARAMETERS
    ---
    area_map : The area map to run the algo on.
    start_points : list of tuples for start points of the drone.
    online : use online if the area is not known by the drone (uses BCD, else STC).
    epochs : number of iterations to run DARP for.
    use_flood : uses traversable l1 distance rather than direct l1 distance.
    drone_speed : None if all drone speeds are the same else ratio.
    drone_coverage : None if all drone coverage areas are the same.
    pbar : If True then shows progress bar.
    """
    n = len(start_points)
    if n > 1:
        A, _ = darp(epochs, area_map, start_points, use_flood=use_flood,
                    pbar=pbar, drone_speed=drone_speed, drone_coverage=drone_coverage)
        drone_maps = [get_drone_map(A, i) for i in range(n)]
    else:
        drone_maps = [area_map]

    if online:
        coverage_paths = [bcd(drone_maps[i], start_points[i])
                          for i in range(n)]
    else:
        coverage_paths = [stc(drone_maps[i], start_points[i])
                          for i in range(n)]
    return coverage_paths
