import time
import numpy as np
import pandas as pd
from tqdm.autonotebook import tqdm

from .metrics import coverage_metrics, fuel_metrics
from .display_funcs import path_show, path_animate, printer
from .helpers import get_random_coords
"""
Test function to run tests on multiple image maps all at once.
"""
def cpp_tests_single_robot_multiple(cpp_algo, area_maps, no_end=False, fuel_paths=True):
    """
    Returns a DataFrame of test Results. 
    Tests are run with randomly generated points for start, end and fuel.
    
    PARAMETERS
    ---
    cpp_algo : the algorithm used to compute the path.
        function signature:
        no_end=True  : `cpp_algo(area_map, start_point)`
        no_end=False : `cpp_algo(area_map, start_point, end_point)`
    area_maps : list of area_map on which the path has to be calculated.
    no_end : if the `cpp_algo` requires an end point then should be true.
    fuel_paths : whether to calculate the fuel paths.
    """
    FC_MAX_DIST_MULTIPLIER = 5
    
    results = []
    for i,area_map in tqdm(enumerate(area_maps), total=len(area_maps)):
        t_metrics = {}
        f_metrics = {}
        start_point = get_random_coords(area_map,1)[0]
        end_point =   get_random_coords(area_map,1)[0]
        
        # Coverage Path calculation.
        t = time.time()
        if no_end:
            coverage_path = cpp_algo(area_map, start_point)
        else:
            coverage_path = cpp_algo(area_map, start_point, end_point)
        t_metrics["cp_compute_time"] = time.time() - t
        c_metrics = coverage_metrics(area_map, coverage_path)
        
        # Fuel Path calculation.
        if fuel_paths == True:
            n_fuel_stations = np.random.randint(1,5)
            fuel_points = get_random_coords(area_map, n_fuel_stations)
            dm = dist_fill(area_map, fuel_points)
            fuel_capacity = dm.max() * FC_MAX_DIST_MULTIPLIER
            
            t = time.time()
            dist_map, _, fuel_paths_, _ = get_fuel_paths(coverage_path, \
                                                             area_map, fuel_points, fuel_capacity)
            t_metrics["fp_compute_time"] = time.time() - t
            f_metrics = fuel_metrics(fuel_paths_, fuel_capacity)
            f_metrics["max_dist_fuel"] = dist_map.max()
        
        results.append({
            **t_metrics,
            **c_metrics,
            **f_metrics
        })
    return pd.DataFrame(results)


"""
Test Function to run tests on a single test map and show the results
"""
def cpp_tests_single_robot_single(cpp_algo, area_map, no_end=False, fuel_paths=True, \
                                  start_point=None, end_point=None, fuel_points=None,\
                                  fuel_capacity=None, fp_count=None, animate=False,\
                                  interval=10, repeat=False,printm=True, show_paths=True,\
                                  figsize=(7,7), cmap="Greys_r"):
    """
    Returns a DataFrame of test Results. 
    Tests are run with randomly generated points for start, end and fuel.
    
    PARAMETERS
    ---
    cpp_algo : the algorithm used to compute the path.
        function signature:
        no_end=True  : `cpp_algo(area_map, start_point)`
        no_end=False : `cpp_algo(area_map, start_point, end_point)`
    area_map : list of area_map on which the path has to be calculated.
    no_end : if the `cpp_algo` requires an end point then should be true.
    fuel_paths : whether to calculate the fuel paths.
    
    --
    If the following values are none random values are used:
        start_point : (x,y) starting point of the drone.
        end_point : (x,y) end point of the drone.
        fuel_points : [(x,y),...,(xn,yn)] fuel points.
        fuel_capacity : drone's fuel capacity.
        fp_count : If fuel_points is None, how many fuel points to generate.
    --
    
    animate : whether to animate the motion of the drone.
    interval : interval in ms between animation frames.
    repeat : whether to repeat the animation.
    printm : whether to print the drone metrics.
    show_paths : show the map.
    fig_size : size of the matplotlib figure to display the animation 
        or area map.
    cmap : Which matplotlib color map to use.
    """
    FC_MAX_DIST_MULTIPLIER = 5
    
    t_metrics = {}
    f_metrics = {}
    
    fuel_paths_ = None
    dist_map = None
    detour_idx = None
    fuel_capacity_list = None
    start_point = get_random_coords(area_map,1)[0] if start_point is None else start_point
    end_point =   get_random_coords(area_map,1)[0] if end_point is None else end_point
    
    # Coverage Path calculation.
    t = time.time()
    if no_end:
        coverage_path = cpp_algo(area_map, start_point)
    else:
        coverage_path = cpp_algo(area_map, start_point, end_point)
    t_metrics["cp_compute_time"] = time.time() - t
    c_metrics = coverage_metrics(area_map, coverage_path)
    
    if fuel_paths == True:
        if fuel_points is None:
            fp_count = np.random.randint(1,5) if fp_count is None else fp_count
            fuel_points = get_random_coords(area_map, fp_count)
            
        if fuel_capacity is None:
            dist_map = dist_fill(area_map, fuel_points)
            fuel_capacity = dist_map.max() * FC_MAX_DIST_MULTIPLIER

        t = time.time()
        dist_map, detour_idx, fuel_paths_, fuel_capacity_list = get_fuel_paths(coverage_path, \
                                                             area_map, fuel_points, fuel_capacity)
        
        t_metrics["fp_compute_time"] = time.time() - t
        f_metrics = fuel_metrics(fuel_paths_, fuel_capacity)
        f_metrics["max_dist_fuel"] = dist_map.max()
        
    metrics = {
        **t_metrics,
        **c_metrics,
        **f_metrics,
    }
    values = {
            "area_map":area_map,
            "coverage_path":coverage_path,
            "fuel_paths":fuel_paths_,
            "detour_idx":detour_idx,
            "dist_map":dist_map,
            "fuel_points":fuel_points,
            "fuel_capacity":fuel_capacity,
            "start_point": start_point,
            "end_point": end_point,
            "fuel_capacity_list": fuel_capacity_list
    }
        
    if not animate and show_paths:
        path_show(area_map, coverage_path, fuel_paths_, dist_map, figsize=figsize,cmap=cmap)
    
    if animate:
        values["__anim__"] = path_animate(values, interval, repeat)
        
    if printm:
        printer(metrics)
        
    return metrics, values