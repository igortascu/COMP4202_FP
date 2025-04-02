import osmnx as ox # type: ignore

ox.settings.use_cache = True
ox.settings.log_console = True

# Get graph
G = ox.graph.graph_from_place("Ottawa, Ontario, Canada", network_type="drive")

# Convert to GeoDataFrames
nodes, edges = ox.graph_to_gdfs(G)

# Save manually
nodes.to_file("data/osm_ottawa_roads/nodes.shp")
edges.to_file("data/osm_ottawa_roads/edges.shp")
