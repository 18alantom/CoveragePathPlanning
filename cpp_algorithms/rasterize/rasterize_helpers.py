import math
from geopy import distance

def get_hv_wh(final_coverage):
    """
    Get haversine calcualted width and height of
    the smallest bounding rectangle of final_coverage.
    """
    llng, llat, rlng, rlat =  final_coverage.bounds
    ll = (llat,llng)
    lr = (llat,rlng)
    tr = (rlat,rlng)
    tl = (rlat,llng)
    w = distance.distance(ll,lr)
    h = distance.distance(ll,tl)
    return w, h

def grid_size(side, final_coverage, returnwh=False):
    """
    side : cell side in meters
    final_coverage : area of coverage polygon
    returnwh : True – returns the haversine 
        w,h of the bounding box 
    """
    w, h = get_hv_wh(final_coverage)
    w_count = math.ceil(w.m/side)
    h_count = math.ceil(h.m/side)
    if returnwh:
        return w_count, h_count, w, h
    return w_count, h_count

def get_bounding_points(final_coverage):
    """
    lower left, top left, top right, lower right
    """
    llng, llat, rlng, rlat =  final_coverage.bounds
    ll = (llat,llng)
    tl = (rlat,llng)
    tr = (rlat,rlng)
    lr = (llat,rlng)
    return {
        'll':ll,
        'tl':tl,
        'tr':tr,
        'lr':lr
    }