import numpy as np
from cpp_algorithms import cpp, conversion
from cpp_algorithms.constants import START, FUEL
from cpp_algorithms.testers.metrics import coverage_metrics


def run_coverage(geojson, altitude=None, fov=None, side=None,
                 epochs=300, online=False,  use_flood=True,
                 fuel_capacity=None, drone_range=None, drone_speed=None,
                 drone_coverage=None, pbar=True):
    """
    PARAMETERS
    ---
    geojson : GeoJson returned from the front end 
    altitude : height of the drone in meters
    fov : field of vision of the drone's camera in degrees
    online : use online if the area is not known by the drone (uses BCD, else STC).
    epochs : number of iterations to run DARP for.
    use_flood : uses traversable l1 distance rather than direct l1 distance.
    fuel_capacity : fuel capacity in units of drone coverage side length
    drone_range : range of the drone
    drone_speed : None if all drone speeds are the same else ratio.
    drone_coverage : None if all drone coverage areas are the same.
    pbar : Progress bar, to show or not to is the quesition.
    """
    if side == None:
        try:
            side = altitude*np.tan(fov/(180/np.pi))
        except:
            raise ValueError("no way to calculate coverage area")
    area_map, points, types, retransformer = conversion(side, geojson)

    fuel_points = None
    try:
        start_points = points[types.index(START)]
    except:
        raise Exception("terrible error :: no start location")

    try:
        fuel_points = points[types.index(FUEL)]
    except:
        print("no fuel points found")

    coverage_paths, full_paths, detour_idxes = cpp(
        area_map, start_points, fuel_points, fuel_capacity, online, epochs, use_flood, drone_speed, drone_coverage, pbar=pbar)
    if full_paths is None:
        full_paths = list(map(np.array, coverage_paths))
    lnglat_path = list(map(retransformer, full_paths))
    lnglat_path = list(map(lambda l: l.reshape(-1, 2), lnglat_path))

    def cm(cp): return coverage_metrics(area_map, cp)
    temp_paths = []
    for x in lnglat_path:
        temp_paths.append(
            list(map(lambda x: {'lat': x[1].item(), 'lng': x[0].item()}, x)))

    lnglat_path = temp_paths

    temp_paths = []
    for x in full_paths:
        temp_paths.append(
            list(map(lambda x: (x[0].item(), x[1].item()), x.reshape(-1, 2))))
    full_paths = temp_paths

    list(map(cm, coverage_paths))

    cov_metrics = []
    for cp in coverage_paths:
        temp = {}
        cm = coverage_metrics(area_map, cp)
        for k in cm:
            try:
                temp[k] = cm[k].item()
            except:
                temp[k] = cm[k]
        cov_metrics.append(temp)

    return {"fullPath": full_paths, "lnglatPath": lnglat_path, "detourIndices": detour_idxes, "coverageMetrics": cov_metrics}
