from cpp_algorithms.darp import darp
from cpp_algorithms.coverage_path.bcd import bcd
from cpp_algorithms.coverage_path.stc import stc
from cpp_algorithms.coverage_path.wavefront import wavefront
from cpp_algorithms.common_helpers import get_drone_map
from cpp_algorithms.fuel_path.fuel_path import get_fuel_paths
from cpp_algorithms.fuel_path.fuel_path_helpers import splice_paths


def cpp(area_map, start_points, fuel_points=None, fuel_capacity=None, online=False, epochs=300, use_flood=True,
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

    if online == "wavefront":
        coverage_paths = [wavefront(drone_maps[i], start_points[i])
                          for i in range(n)]
    elif online:
        coverage_paths = [bcd(drone_maps[i], start_points[i])
                          for i in range(n)]
    else:
        coverage_paths = [stc(drone_maps[i], start_points[i])
                          for i in range(n)]
    if fuel_points is not None and fuel_capacity is not None:
        full_paths = []
        detour_idxes = []
        for coverage_path in coverage_paths:
            # Get fuel path
            _, detour_idx, fuel_paths, _ = get_fuel_paths(
                coverage_path, area_map, fuel_points, fuel_capacity)
            # Splice fuel path
            full_path, detour_idx = splice_paths(
                coverage_path, fuel_paths, detour_idx)
            full_paths.append(full_path)
            detour_idxes.append(detour_idx)
        return coverage_paths, full_paths, detour_idxes
    return coverage_paths, None, None
