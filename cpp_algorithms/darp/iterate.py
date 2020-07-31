import numpy as np
from tqdm.auto import tqdm

from cpp_algorithms.constants import OB

from .darp_helpers import get_assignment_matrix, get_assigned_count
from .darp_helpers import rmse, get_fair_share
from .continuity import continuity_check_fix


"""
loop functions
"""
def apply_mask_E(mask, E, obstacle=OB):
    E[:, mask] = obstacle
    
def mi_step(start_points, E, m, c, nobs_count, coverage_ratio, flood_matrix, obstacle=OB):
    """
    start_points : drone start coords on the map
    E : evaluation matrices for all drones
    m : array of m_i values 
    c : tunable parameter
    nobs_count : total area that can be covered
    coverage_ratio : coverage ratio for all drones
    flood_matrix : used in Ci calculation
    """
    n = E.shape[0]
    mask = E[0] == obstacle
    A = get_assignment_matrix(E) # E changes ∴ A changes
    for i in range(len(m)):
        start_point = start_points[i]
        assigned_count = get_assigned_count(A, arr=True) # Counts of cells assigned to each drone
        fs_i = get_fair_share(i, assigned_count, nobs_count, coverage_ratio) # Difference between optimal and assigned
        m[i] = 1 + c * fs_i # alter the m param
        E[i] = E[i] * m[i]  # update evaluation matrix
        apply_mask_E(mask, E)
        A = get_assignment_matrix(E) # E changes ∴ A changes
        continuity_check_fix(A, E, i, n, m[i], mask, start_point, flood_matrix)
    return A

def iterate(epochs, start_points, E, c, nobs_count, coverage_ratio, flood_matrix=None, print_stuff=False, pbar=True, obstacle=OB):
    """
    epochs : number of iterations to iterate the loop in the iterate function for.
    start_points : drone start coords on the map
    E : initial evaluation matrix for all the drones
    c : tunable paramter `c`
    nobs_count : number of cells that can be covered
    coverage_ratio : ratio of coverage
    flood_matrix : used in Ci calculation
    print_stuff : print some metrics
    pbar : whether to show the pbar
    """
    n = E.shape[0]
    m = np.ones(n, dtype=np.float32)
    optimal_count = np.round(coverage_ratio * nobs_count)
    best_A = get_assignment_matrix(E)
    min_loss = float('inf')
    losses = []
    for i in tqdm(range(epochs), disable=not pbar):
        A = mi_step(start_points, E, m, c, nobs_count, coverage_ratio, flood_matrix)
        assigned_count = get_assigned_count(A)
        loss = rmse(assigned_count, optimal_count)
        losses.append(loss)
        if print_stuff:
            print(f"ASGN : {assigned_count}\nRMSE : {loss:0.2f}")
        if loss < min_loss:
            min_loss = loss
            best_A = A
    return best_A, losses
