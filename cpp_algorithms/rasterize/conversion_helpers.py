from .rasterize import rasterize

KEY = "type"
AREAS = ["coverage", "obstacle"]
POINTS = ["drone", "fuel"]
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
  
def conversion(side, shapefiles):
	"""
	side : drone area of coverage square's side in meters.
	shapefiles : list of geopandas dataframes 
		created from shapefiles.
	"""
	features = get_features_dict(shapefiles)
	final_coverage = get_final_coverage_polygon(features)
	points = get_points_dict(features)
	area_map, imp_points = rasterize(100, final_coverage, points['points'])
	return area_map, imp_points

