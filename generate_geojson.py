import json
import pyproj

# home of the 1st drone (0,0) in meters used in path generation
HOME_LAT = 45.478
HOME_LON = 9.227

# vertices in meters
vertici_relativi = [
    (3.0, 25.0), 
    (25.0, 9.0), 
    (45.0, 19.0), 
    (18.0, 58.0), 
    (-10.0, 40.0)
]
# first and last points have to be the same to close the shape in a Geojson polygone
vertici_relativi.append(vertici_relativi[0])

proj_local = pyproj.Proj(proj='aeqd', lat_0=HOME_LAT, lon_0=HOME_LON, datum='WGS84', units='m')

coordinate_gps = []
for x, y in vertici_relativi:
    lon, lat = proj_local(x, y, inverse=True)
    coordinate_gps.append([lon, lat])

# standard geojson structure
standard_geojson = {
    "type": "FeatureCollection",
    "features": [{
        "type": "Feature",
        "geometry": {
            "type": "Polygon",
            "coordinates": [coordinate_gps]
        }
    }]
}

with open("disaster_area.geojson", 'w') as f:
    json.dump(standard_geojson, f, indent=4)
    
print("file disaster_area.geojson created")