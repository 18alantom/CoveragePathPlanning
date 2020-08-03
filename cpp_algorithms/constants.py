OB = -1 # Obstacle value, don't change!     
FU = 0 # Value of fuel on the fuel dist_map
NO = 0 # Area to be mapped value

# Conversion constants
START = "drone"
FUEL = "fuel"

# GeoJSON Keys
KEY = "type"
COVERAGE = "coverage"
OBSTACLE = "obstacle"
AREAS = [COVERAGE, OBSTACLE]
POINTS = [START, FUEL]
FEATURES = [*AREAS, *POINTS]
POINT = "layers/POINT"
POLYG = "layers/POLYGON"

# Proojection constants
EPSG = 4326
CRS = f"EPSG:{EPSG}"

# Geometry
GEOM_POINT = "Point"
GEOM_POLYG = "Polygon"