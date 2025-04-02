import time
import pandas as pd
import geopandas as gpd
import networkx as nx
from geopy.distance import geodesic
from algorithms import build_graph_from_shapefile, find_nearest_node, dijkstra_route, astar_route, dijkstra_algorithm, astar_algorithm

# -------------------- Settings -------------------- #
weather = "clear"
traffic = "moderate"
avg_speed_kmph = 70

# -------------------- ETA Helpers -------------------- #
def estimate_eta(distance_meters, avg_speed_kmph=70, weather="", traffic=""):
    speed_mod = 1.0
    if weather == "snow":
        speed_mod -= 0.3
    elif weather == "rain":
        speed_mod -= 0.15
    if traffic == "heavy":
        speed_mod -= 0.25
    elif traffic == "moderate":
        speed_mod -= 0.10

    adjusted_speed = max(avg_speed_kmph * speed_mod, 5)
    speed_m_per_min = (adjusted_speed * 1000) / 60
    return distance_meters / speed_m_per_min

def format_eta(eta_minutes):
    mins = int(eta_minutes)
    secs = int((eta_minutes - mins) * 60)
    return f"{mins} min {secs} sec"

# -------------------- Load Map + Data -------------------- #
edges_gdf = gpd.read_file("data/osm_ottawa_roads/edges.shp")
nodes_gdf = gpd.read_file("data/osm_ottawa_roads/nodes.shp")
G = build_graph_from_shapefile(edges_gdf)

for _, row in nodes_gdf.iterrows():
    G.nodes[row['osmid']]['coords'] = (row['y'], row['x'])

ambulance_df = pd.read_csv("data/ambulances.csv")
incident_df = pd.read_csv("data/incidents.csv")

# Filter only available ambulances
available_ambulances = ambulance_df[ambulance_df['status'] == 'available'].copy()

# -------------------- Main Dispatch Function -------------------- #
def dispatch_to_incidents(selected_incident_ids):
    assigned_ambulance_ids = set()

    for incident_id in selected_incident_ids:
        incident = incident_df[incident_df["id"] == incident_id].iloc[0]
        incident_point = (incident["lat"], incident["lon"])
        incident_node = find_nearest_node(G, incident_point)

        best_result = None

        for _, amb in available_ambulances.iterrows():
            if amb["id"] in assigned_ambulance_ids:
                continue

            amb_point = (amb["lat"], amb["lon"])
            amb_node = find_nearest_node(G, amb_point)

            # --- Dijkstra
            d_start = time.time()
            d_path = dijkstra_route(G, amb_node, incident_node)
            d_time = time.time() - d_start
            d_length = sum(G[u][v]['weight'] for u, v in zip(d_path[:-1], d_path[1:]))
            d_eta = estimate_eta(d_length, avg_speed_kmph, weather, traffic)

            # --- A*
            a_start = time.time()
            a_path = astar_route(G, amb_node, incident_node)
            a_time = time.time() - a_start
            a_length = sum(G[u][v]['weight'] for u, v in zip(a_path[:-1], a_path[1:]))
            a_eta = estimate_eta(a_length, avg_speed_kmph, weather, traffic)

            result = {
                "Ambulance ID": amb["id"],
                "Ambulance Name": amb["name"],
                "Latitude": amb["lat"],
                "Longitude": amb["lon"],
                "Dijkstra Path Length (m)": d_length,
                "Dijkstra ETA (min)": d_eta,
                "Dijkstra ETA (formatted)": format_eta(d_eta),
                "Dijkstra Execution Time (s)": d_time,
                "A* Path Length (m)": a_length,
                "A* ETA (min)": a_eta,
                "A* ETA (formatted)": format_eta(a_eta),
                "A* Execution Time (s)": a_time
            }

            # Track best by shortest ETA 
            if best_result is None or result["Dijkstra ETA (min)"] < best_result["Dijkstra ETA (min)"]:
                best_result = result

        if best_result:
            assigned_ambulance_ids.add(best_result["Ambulance ID"])

            print(f"\n#----------- Incident ID: {incident_id} — Severity: {incident['type']} -----------#")
            print(f"Location: ({incident['lat']}, {incident['lon']})")
            print(f"Conditions [ Weather: {weather.capitalize()}, Traffic: {traffic.capitalize()}, Avg Speed: {avg_speed_kmph} km/h ]")


            print(f"\n----- Assigned Ambulance (Dijkstra) -----")
            print(f"Ambulance: {best_result['Ambulance Name']}")
            print(f"Location: ({best_result['Latitude']}, {best_result['Longitude']})")
            print(f"Path Length: {best_result['Dijkstra Path Length (m)']:.2f} m")
            print(f"ETA: {best_result['Dijkstra ETA (formatted)']}")
            print(f"\nAlgorithm Execution Time: {best_result['Dijkstra Execution Time (s)']:.4f} s")

            print(f"\n----- Assigned Ambulance (A*) -----")
            print(f"Ambulance: {best_result['Ambulance Name']}")
            print(f"Location: ({best_result['Latitude']}, {best_result['Longitude']})")
            print(f"Path Length: {best_result['A* Path Length (m)']:.2f} m")
            print(f"ETA: {best_result['A* ETA (formatted)']}")
            print(f"\n Algorithm Execution Time: {best_result['A* Execution Time (s)']:.4f} s")
        else:
            print(f"\n No available ambulance for Incident {incident_id}")

# -------------------- Run Dispatch -------------------- #
dispatch_to_incidents([101, 102, 103])
