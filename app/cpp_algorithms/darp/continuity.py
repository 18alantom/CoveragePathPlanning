import numpy as np
from tqdm.auto import tqdm
from skimage import measure
from cpp_algorithms.constants import OB
from cpp_algorithms.dist_fill import dist_fill

"""
Functions to calculate the continuity correction matrix.
"""
def get_area_indices(area, value, inv=False, obstacle=OB):
    """
    returns indices that have value and not obstacle
    if inv returns indices that don't have value and obstacle
    """
    try:
        value = int(value)
        if inv:
            return np.concatenate([np.where((area != value)&(area!=obstacle))]).T
        return np.concatenate([np.where((area == value)&(area!=obstacle))]).T
    except:
        mask = area == value[0]
        if inv:
            mask = area != value[0]
        for v in value[1:]:
            if inv:
                mask &= area != v
            else:
                mask |= area == v
        mask &= area != obstacle
        return np.concatenate([np.where(mask)]).T

def get_flood_matrix(area_map, pbar=False, obstacle=OB):
    """
    Returns all the matrices of distances
    """
    points = get_area_indices(area_map, obstacle, True)
    flood_matrix = []
    for point in tqdm(points,disable=not pbar):
         flood_matrix.append(dist_fill(area_map, [tuple(point)]))
    return  np.array(flood_matrix)

def get_region_dist_map(index_matrix, coords):
    """
    Broadcasts coords along index_matrix and returns the min l1
    """
    b_val = index_matrix.shape[0]
    coords = np.broadcast_to(coords, (b_val, *coords.shape))
    assert coords.shape[0] == index_matrix.shape[0] and \
        coords.shape[2] == index_matrix.shape[2], \
        "something went wrong in broadcasting"
    
    # To change the distance function change this line.
    return np.abs(index_matrix - coords).sum(axis=2).min(axis=1)

def get_ci(cont_map, r, q):
    """
    cont_map : continuity map that assigns unique labels to every isolated !obstacle
    r : value of region with the start point
    c : value(s) of region without the start point
    """
    index_matrix = np.indices(cont_map.shape).transpose(1,2,0).reshape(-1, 2)[:,None,:]
    r_indices = get_area_indices(cont_map, r, obstacle=0)
    q_indices = get_area_indices(cont_map, q, obstacle=0)

    min_dist_r = get_region_dist_map(index_matrix, r_indices).reshape(cont_map.shape)
    min_dist_q = get_region_dist_map(index_matrix, q_indices).reshape(cont_map.shape)
    return min_dist_r - min_dist_q
    
def min_point_map(cont_map, value, flood_matrix, obstacle=OB):
    """
    Returns the min value to a region.
    """
    all_idx = get_area_indices(cont_map, obstacle, True, obstacle).T
    val_idx = get_area_indices(cont_map, value, obstacle=obstacle).T
    min_map = cont_map.copy()
    ax,ay = all_idx
    vx,vy = val_idx
    min_map[ax,ay] = flood_matrix[:,vx,vy].min(axis=1)
    return min_map
    
def get_ci_using_flood_matrix(cont_map, r, q, flood_matrix):
    """
    Returns Ci calculated from the flood matrix
    """
    min_dist_r = min_point_map(cont_map, r, flood_matrix, obstacle=0)
    min_dist_q = min_point_map(cont_map, q, flood_matrix, obstacle=0)
    return min_dist_r - min_dist_q

# Functions that deal with continuity.
def coord_in_list(coord, list_):
    """
    Checks if coord is present in the list exactly once.
    """
    return (np.abs(np.array(coord) - list_).sum(axis=1) ==0).sum() == 1

def continuity_check(A, n, i):
    """
    True if all areas are continuous.
    A : assignment map
    n : number of drones
    """
    cont_map, cont_count = measure.label(A, return_num=True, background=-1)
    xy = get_area_indices(A,i)
    x,y = xy.T
    values_at_xy = cont_map[x,y]
    not_is_cont = len(np.unique(values_at_xy)) > 1
    if not_is_cont:
        return False, xy, values_at_xy, cont_map
    else:
        return True, None, None, None

def continuity_fix(A, E, i, n, mi, start_point, mask, xy, values_at_xy, cont_map, flood_matrix):
    """
    Fix continuity when there is none.
    """
    r = None
    q = None
    some_constant = 10**-np.log10(A.size)
    uniq = np.unique(values_at_xy)
    for v in uniq:
        if coord_in_list(start_point,xy[values_at_xy == v]):
            r = v
            break
    q = uniq[uniq!=r]
    if flood_matrix is None:
        C_i = get_ci(cont_map, r, q)
    else:
        C_i = get_ci_using_flood_matrix(cont_map, r, q, flood_matrix)
        
    C_i[mask] = 1
    E[i] = E[i]*(C_i * some_constant + 1)
    
def continuity_check_fix(A, E, i, n, mi, mask, start_point, flood_matrix):
    """
    Calculate C_i and fix if `i`th drone's areas are not contiguous.
    """
    is_contig, xy, values_at_xy, cont_map = continuity_check(A, n, i)
    if is_contig:
        return
    else:
        continuity_fix(A, E, i, n, mi, start_point, mask, xy, values_at_xy, cont_map, flood_matrix)