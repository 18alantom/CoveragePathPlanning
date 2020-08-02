import numpy as np
import geopy
import shapely

from .rasterize_helpers import grid_size, get_bounding_points
def grid_points(side, coverage_area):
    """
    side : drone coverage square side
    coverage_area : area to be covered by the drone
    """
    width, height = grid_size(side, coverage_area)
    side = geopy.distance.distance(meters=side)
    bpoints = get_bounding_points(coverage_area)
    

    latitudes = []
    start = geopy.Point(bpoints['ll'])
    for _ in range(height):
        latitudes.append(start)
        final = side.destination(point=start, bearing=0)
        start = final
    latitudes.append(start)
    
    longitudes = []
    start = geopy.Point(bpoints['ll'])
    for _ in range(width):
        longitudes.append(start)
        final = side.destination(point=start, bearing=90)
        start = final
    longitudes.append(start)
    
    return latitudes, longitudes, height, width

def rasterize(side, coverage_area, imp_points):
    """
    side : side of the drone in meters
    coverage_area : area to be covered by the drone
    imp_points : list of important points
    """
    
    outside_map_traverse_area = coverage_area.envelope.difference(coverage_area)
    latitudes,longitudes,height,width = grid_points(side, coverage_area)
    lookup_c = np.zeros([height,width],dtype = 'O')
    raster = np.zeros([height, width],dtype = int)
    imp_coords = []
    
    
    for x in range(len(latitudes)-1):
        for y in range(len(longitudes)-1):
            square = shapely.geometry.Polygon([(longitudes[y][1],latitudes[height-x][0]),\
                              (longitudes[y+1][1],latitudes[height-x][0]),\
                              (longitudes[y+1][1],latitudes[height-x-1][0]),\
                              (longitudes[y][1],latitudes[height-x-1][0])]) 
            for imp_point in imp_points:
                if(imp_point.within(square)):
                    imp_coords.append((x,y))
                    
            lookup_c[x,y] = square.centroid
            # storing value of centre point
            if(outside_map_traverse_area.contains(square)):
                raster[x,y] = -1  # region outside the map but inside the gird
            if(square.intersects(coverage_area)):
                    z = square.intersection(coverage_area)
                    if(not z.is_empty):
                        if(coverage_area.contains(z)):
                            raster[x,y] = 0 #region inside the map
                        else:
                            raster[x,y] = 0 #2 is for boundary partial
                            
    raster[0] = -1
    raster[:,-1] = -1
    return raster, imp_coords, lookup_c