"""
Some helper functions that are often used.
"""
import numpy as np
import matplotlib.pyplot as plt

def imshow(area_map,figsize=(5,5),cmap="viridis"):
    """
    Display with no interpolation.
    """
    plt.figure(figsize=figsize)
    ax = plt.imshow(area_map,interpolation='none',cmap=cmap)
    plt.axis('off');
    return ax

def get_random_coords(area_map,n=2,obs=-1):
    """
    Return random coords from the map
    where there are no obstacles.
    
    n : number of random points
    obs : obstacle value on the area map
    """
    r = lambda x : np.random.randint(0,x)
    b1, b2 = area_map.shape
    coords = []
    for i in range(n):
        while True:
            p = (r(b1), r(b2))
            if area_map[p] != obs:
                coords.append(p)
                break
    return coords

def set_val(area_map, coords, val):
    """
    Set `val` at given `coords` on
    the `area_map`
    
    area_map : 2D numpy array
    coords : list of (x,y) tuples
    val : int of value to set
    """
    x, y = np.array(coords).T
    area_map[x,y] = val

def is_bounded(coord, shape):
    """
    Checks if a coord (x,y) is within bounds.
    """
    x,y = coord
    g,h = shape
    lesser = x < 0 or y < 0
    greater = x >= g or y >= h
    if lesser or greater:
        return False
    return True

def is_valid(coord, area_map, obstacle = -1):
    """
    Check is a coord (x,y) is bounded and not
    on an obstacle.
    """
    is_b = is_bounded(coord, area_map.shape)

    is_on_obs  = area_map[coord] == obstacle
    if is_b and not is_on_obs:
        return True
    return False