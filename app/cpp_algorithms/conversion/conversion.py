from .conversion_helpers import get_scale, get_gpdframe
from .conversion_helpers import create_gdframe
from .conversion_helpers import get_features_dict, get_final_coverage_polygon
import json
import shapely
import geopandas as gpd
import numpy as np
from geopy import distance
from skimage import measure
from skimage.draw import polygon
from cpp_algorithms.constants import CRS, EPSG, KEY, GEOM_POINT


def get_raster(gpdf_final, scale=2000, CRS=CRS):
    assert len(gpdf_final) == 1
    try:
        shp = gpdf_final.to_crs(crs=CRS)
    except:
        shp = gpdf_final.set_crs(crs=CRS)

    ext = np.array(shp.geometry[0].exterior).copy()
    ite = map(np.array, shp.geometry[0].interiors)

    mn = ext.min(axis=0)
    mix = ext - mn
    mx = mix.max()
    mix *= scale/mx
    mix = np.int64(mix)
    sh = mix.max(axis=0)

    r, c = polygon(*mix.T, sh)
    p = np.full(mix.max(axis=0), -1)
    p[r, c] = 0

    for o in ite:
        r, c = polygon(*np.int16((o-mn)*scale/mx).T, sh)
        p[r, c] = -1

    return p, mn, mx


def coo_to_points(gpdf_points, mn, mx, key=KEY, scale=2000):
    types = []
    points = []
    for p in gpdf_points.iterrows():
        if p[1][key] not in types:
            types.append(p[1][key])
            points.append([])

        i = types.index(p[1][key])
        coords = np.array(p[1].geometry.coords)
        points[i].append(tuple(np.int64((coords - mn)*scale/mx)[0]))
    return points, types


def down_sample(side, area_map, points, meter=1):
    st = int(side/meter)
    area_map = area_map.copy()
    area_map[area_map == -1] = 1
    vals = []
    for i, point in enumerate(points):
        point = np.array(point)
        x, y = point.T
        area_map[x, y] = i+2
        vals.append(i+2)

    temp = measure.block_reduce(area_map, (st, st), np.max, cval=0)
    temp[temp == 1] = -1
    points = []
    for val in vals:
        points_ = np.stack(np.where(temp == val)).T
        points.append(list(map(tuple, points_)))
        temp[temp == val] = 0
    return temp, points


def conversion(side, geojson):
    """
    side : drone area of coverage square's side in meters.
    geo_json : parsed json in the geojson fromat from the frontend.
    """
    if isinstance(geojson, str):
        geojson = json.loads(geojson)

    # contains the GeopandasDataFrames for all featureCollections
    gpdf_all = get_gpdframe(geojson)
    features = get_features_dict(gpdf_all)

    # Unnecessary repitition someone improve it
    gpdf_final = create_gdframe(
        features, no_points=True).set_crs(epsg=EPSG)
    final_coverage_polygon = gpdf_final.geometry[0]

    if gpdf_all[0].geom_type[0] == GEOM_POINT:
        gpdf_points = gpdf_all[0]
    else:
        gpdf_points = gpdf_all[1]
    gpdf_points = gpdf_points.set_crs(epsg=EPSG)

    # If the scale is large it takes a lot of time to compute
    m = 1
    scale = get_scale(final_coverage_polygon, meter=m)
    while scale > 3000:
        m *= 2
        scale = get_scale(final_coverage_polygon, meter=m)

    area_map_r, mn, mx = get_raster(gpdf_final, scale, CRS=CRS)
    points_r, types = coo_to_points(gpdf_points, mn, mx, scale=scale)
    area_map, points = down_sample(side, area_map_r, points_r, meter=m)

    # print(mn, mx, scale, area_map.shape, m, area_map_r.shape)
    def retransformer(cp): return ((np.array(cp)*side) *
                                   mx/(scale * m))+mn[None, None, :] + 1e-4

    return area_map, points, types, retransformer
