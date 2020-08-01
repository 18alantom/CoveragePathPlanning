from .rasterize import rasterize

KEY = "type"
AREAS = ["coverage", "obstacle"]
POINTS = ["drone", "fuel"]
POINT = "Point"
POLYG = "Polygon"
FEATURES = [*AREAS, *POINTS]

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
    final = features['coverage'][0]
    for cov in features['coverage']:
        final = final.union(cov)
    for obs in features['obstacle']:
        final = final.difference(obs)
    return final

def get_points_dict(features, points_keys = ['drone', 'fuel']):
    points = {
        "type":[],
        "points":[]
    }
    for key in points_keys:
        for point in features[key]:
            points['type'].append(key)
            points['points'].append(point)
    return points

def get_gpdframe(geo_json):
    # Change all constantns
    gpdframe = []
    for gj in geo_json:
        temp = []
        for feature in gj['features']:
            if gj['fileName'] == POINT:
                geom = shapely.geometry.Point(feature['geometry']['coordinates'])
            elif gj['fileName'] == POLYG:
                assert len(feature['geometry']['coordinates']) == 1, "yo there be more stuff here 👈"
                geom = shapely.geometry.Polygon(feature['geometry']['coordinates'][0])
            temp.append({
                KEY: feature['properties']['type'],
                'geometry': geom
            })
        gpdframe.append(geopandas.GeoDataFrame(temp))
    return gpdframe
  
def conversion(side, geo_json):
	"""
	side : drone area of coverage square's side in meters.
	geo_json : parsed json in the geojson fromat from the frontend.
	"""
	shapefiles = get_gpdframe(geo_json)
	features = get_features_dict(shapefiles)
	final_coverage = get_final_coverage_polygon(features)
	points = get_points_dict(features)
	area_map, imp_points = rasterize(side, final_coverage, points['points'])
	return area_map, imp_points