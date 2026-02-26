import json
import math
import pyproj
import os
import glob
import argparse
from shapely.geometry import Polygon, LineString

# gps management and conversion
def load_geojson_and_convert(geojson_path, home_lat, home_lon):
    proj_local = pyproj.Proj(proj='aeqd', lat_0=home_lat, lon_0=home_lon, datum='WGS84', units='m')
    
    with open(geojson_path, 'r') as f:
        data = json.load(f)
        
    try:
        gps_coords = data['features'][0]['geometry']['coordinates'][0]
    except KeyError:
        print("Invalid GeoJSON. No polygon found")
        return []

    local_vertices = []
    for lon, lat in gps_coords:
        x, y = proj_local(lon, lat)
        local_vertices.append((x, y))
        
    return local_vertices

# create lawnmower path
def generate_polygon_lawnmower(vertices, altitude, sensor_radius, side_overlap, forward_overlap):
    poly = Polygon(vertices)
    minx, miny, maxx, maxy = poly.bounds

    diameter = 2.0 * sensor_radius
    step_side = diameter * (1.0 - side_overlap)  
    step_forward = diameter * (1.0 - forward_overlap)

    mission_points = []
    current_y = miny + sensor_radius
    going_right = True

    while current_y <= maxy:
        sweep_line = LineString([(minx - 10, current_y), (maxx + 10, current_y)])
        intersection = sweep_line.intersection(poly)
        
        if intersection.is_empty:
            current_y += step_side 
            continue
            
        if intersection.geom_type == 'LineString':
            seg_minx, _, seg_maxx, _ = intersection.bounds
            
            row_points = []
            curr_x = seg_minx + sensor_radius
            while curr_x <= seg_maxx - sensor_radius:
                row_points.append((curr_x, current_y, altitude))
                curr_x += step_forward
                
            if not going_right:
                row_points.reverse()
                
            mission_points.extend(row_points)
            if len(row_points) > 0:
                going_right = not going_right
                
        current_y += step_side

    if len(mission_points) > 0:
        dist_to_first = math.hypot(mission_points[0][0], mission_points[0][1])
        dist_to_last = math.hypot(mission_points[-1][0], mission_points[-1][1])
        if dist_to_last < dist_to_first:
            mission_points.reverse() 

    return mission_points

# handle swarm and battery logic
def get_home_pos(drone_id):
    idx = drone_id - 1 
    x = 0.0 if idx < 5 else 2.0  
    y = float((idx % 5) * 2.0)   
    return (x, y)

def setup_output_folder(folder_name):
    if not os.path.exists(folder_name):
        os.makedirs(folder_name)
        print(f"folder created: {folder_name}/")
    else:
        # clear previous missions
        vecchi_file = glob.glob(os.path.join(folder_name, "mission_drone*.txt"))
        for f in vecchi_file:
            os.remove(f)
        print(f"old mission files removed")

def save_mission_file(drone_id, part, waypoints, altitude, output_dir):
    hx, hy = get_home_pos(drone_id) 
    
    # save new files
    filename = os.path.join(output_dir, f"mission_drone{drone_id}_part{part}.txt")
    with open(filename, "w") as f:
        f.write(f"{hx:.2f},{hy:.2f},{altitude:.2f}\n") 
        for wp in waypoints:
            f.write(f"{wp[0]:.2f},{wp[1]:.2f},{wp[2]:.2f}\n")
        f.write(f"{hx:.2f},{hy:.2f},{altitude:.2f}\n") 
        f.write(f"{hx:.2f},{hy:.2f},0.10\n") 
    return filename

def generate_swarm_missions(all_waypoints, num_drones, altitude, drone_specs, wind_speed, output_dir):
    total_wp = len(all_waypoints)
    if total_wp == 0:
        return

    chunk_size = math.ceil(total_wp / num_drones)
    wind_penalty_factor = 1.0 + (wind_speed * 0.05) 
    
    print(f"\Swarm: {total_wp} waypoints divided by {num_drones} drones.")
    print(f"wind speed: {wind_speed} m/s (wind penalty factor: {wind_penalty_factor:.2f}x)")
    
    for i in range(num_drones):
        drone_id = i + 1
        drone_wps = all_waypoints[i*chunk_size : (i+1)*chunk_size]
        if not drone_wps:
            continue
            
        max_battery_mins = drone_specs.get(drone_id, {}).get('battery_mins', 15.0)
        drone_speed = drone_specs.get(drone_id, {}).get('speed_ms', 3.0)
            
        part = 1
        current_mission_wps = []
        current_drain_mins = 0.0
        
        hx, hy = get_home_pos(drone_id)
        last_pos = (hx, hy) 
        
        for wp in drone_wps:
            step_dist = math.hypot(wp[0]-last_pos[0], wp[1]-last_pos[1])
            rtl_dist = math.hypot(wp[0]-hx, wp[1]-hy) 
            
            step_drain = ((step_dist / drone_speed) / 60.0) * wind_penalty_factor
            rtl_drain = ((rtl_dist / drone_speed) / 60.0) * wind_penalty_factor
            
            if current_drain_mins + step_drain + rtl_drain > max_battery_mins and len(current_mission_wps) > 0:
                print(f"Drone {drone_id}: Battery problems ({current_drain_mins:.1f}/{max_battery_mins} min used). Pit stop number {part}.")
                # output directory
                save_mission_file(drone_id, part, current_mission_wps, altitude, output_dir)
                
                part += 1
                current_mission_wps = []
                current_drain_mins = 0.0
                last_pos = (hx, hy) 
                
                step_dist = math.hypot(wp[0]-hx, wp[1]-hy)
                step_drain = ((step_dist / drone_speed) / 60.0) * wind_penalty_factor
            
            current_mission_wps.append(wp)
            current_drain_mins += step_drain
            last_pos = (wp[0], wp[1])
            
        if current_mission_wps:
            filename = save_mission_file(drone_id, part, current_mission_wps, altitude, output_dir)
            print(f"Drone {drone_id} (part {part}): mission created. saved in {filename}.")

if __name__ == "__main__":
    # arg paser from terminal
    parser = argparse.ArgumentParser(description="Multi-Drone mission generator for SAFE project.")
    parser.add_argument('-n', '--num_drones', type=int, default=3, help="number of drones in the swarm (default: 3)")
    parser.add_argument('-a', '--altitude', type=float, default=15.0, help="heights of the mission (default: 15.0)")
    parser.add_argument('-r', '--sensor_radius', type=float, default=2.0, help="radious of the antenna (default: 2.0)")
    parser.add_argument('-w', '--wind_speed', type=float, default=4.5, help="wind speed in m/s (default: 4.5)")
    parser.add_argument('-o', '--output_dir', type=str, default="mission_paths", help="output folder for mission paths (default: mission_paths)")
    
    args = parser.parse_args()

    HOME_LAT = 45.478
    HOME_LON = 9.227
    
    # hardware drone specs
    DRONE_SPECS = {
        1: {'battery_mins': 20.0, 'speed_ms': 4.0},
        2: {'battery_mins': 25.0, 'speed_ms': 3.0},
        3: {'battery_mins': 18.0, 'speed_ms': 3.0},
        4: {'battery_mins': 30.0, 'speed_ms': 5.0},
        5: {'battery_mins': 25.0, 'speed_ms': 3.0},
        6: {'battery_mins': 15.0, 'speed_ms': 4.0},
        7: {'battery_mins': 35.0, 'speed_ms': 4.0},
        8: {'battery_mins': 17.0, 'speed_ms': 5.0},
        9: {'battery_mins': 22.0, 'speed_ms': 6.0},
        10: {'battery_mins': 20.0, 'speed_ms': 2.0}
    }
    
    setup_output_folder(args.output_dir)

    geojson_filename = "simulation/disaster_area.geojson"
    if not os.path.exists(geojson_filename):
        print(f"File {geojson_filename} not found.")
        exit(1)

    poligono_metrico = load_geojson_and_convert(geojson_filename, HOME_LAT, HOME_LON)
    
    if poligono_metrico:
        print(f"GeoJSON loaded. Mission area contains {len(poligono_metrico)} vertices")
        
        punti_generati = generate_polygon_lawnmower(
            vertices=poligono_metrico,
            altitude=args.altitude,        
            sensor_radius=args.sensor_radius,    
            side_overlap=0.20,    
            forward_overlap=0.30  
        )
        
        generate_swarm_missions(punti_generati, args.num_drones, args.altitude, DRONE_SPECS, args.wind_speed, args.output_dir)