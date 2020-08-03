import json
import numpy as np
import shapely.geometry
import geopandas as gpd
from geopy import distance
from cpp_algorithms.constants import KEY, COVERAGE, OBSTACLE, POINT
from cpp_algorithms.constants import AREAS, POINTS, FEATURES, POLYG
from cpp_algorithms.constants import CRS, GEOM_POINT, GEOM_POLYG


def get_features_dict(shapefiles, key=KEY, fnames=FEATURES):
    """
    Name of the features should be shapefile
    `key` column values
    """
    features = {}
    for name in fnames:
        features[name] = []

    for sh in shapefiles:
        for rows in sh.iterrows():
            for k in features:
                if rows[1][key].find(k) >= 0:
                    features[k].append(rows[1].geometry)
    return features


def get_final_coverage_polygon(features):
    """
    Union of coverage and difference of
    obstacles.
    """
    final = features[COVERAGE][0]
    for cov in features[COVERAGE]:
        final = final.union(cov)
    for obs in features[OBSTACLE]:
        final = final.difference(obs)
    return final


def get_gpdframe(geojson):
    if isinstance(geojson, list):
        # Takes care of : [FeatureCollection,...,FeaturesCollection]
        return list(map(gpd.GeoDataFrame.from_features, geojson))
    else:
        # Geojson type not property type.
        if geojson['type'] == "FeatureCollection":
            all_ = gpd.GeoDataFrame.from_features(geojson['features'])
            return [
                all_[all_.geom_type == GEOM_POINT],
                all_[all_.geom_type == GEOM_POLYG]
            ]
        else:
            # Might break the code ahead
            return gpd.GeoDataFrame.from_features(geojson)


# Terrible naming conventions  I know.
def create_gdframe(features, crs=CRS, no_points=False):
    """
    Create GeoDataFrame from features
    """
    final_coverage = get_final_coverage_polygon(features)
    points = []
    if not no_points:
        for d in features[START]:
            points.append({
                KEY: START,
                'geometry': d
            })
        for f in features[FUEL]:
            points.append({
                KEY: FUEL,
                'geometry': d
            })
    points.append({
        KEY: COVERAGE,
        'geometry': final_coverage})
    return gpd.GeoDataFrame(points, crs=crs)


def get_hv_wh(final_coverage_polygon):
    """
    Get haversine calcualted width and height of
    the smallest bounding rectangle of the coverage area.
    """
    llng, llat, rlng, rlat = final_coverage_polygon.bounds
    ll = (llat, llng)
    lr = (llat, rlng)
    tr = (rlat, rlng)
    tl = (rlat, llng)
    w = distance.distance(ll, lr)
    h = distance.distance(ll, tl)
    return w, h


def get_scale(final_coverage_polygon, meter=1):
    """
    meter : 1 pixel == ? meters
    """
    w, h = get_hv_wh(final_coverage_polygon)
    w = w.m
    h = h.m
    return int(np.round((np.array([w, h])/meter).max()))
