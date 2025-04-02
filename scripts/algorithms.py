import networkx as nx
from geopy.distance import geodesic
import heapq

def haversine(coord1, coord2):
    """Calculate the Haversine distance between two (lat, lon) points in meters."""
    return geodesic(coord1, coord2).meters

def build_graph_from_shapefile(edges_gdf):
    """
    Convert a GeoDataFrame of road segments into a NetworkX graph.
    Assumes the GeoDataFrame contains LineStrings with 'u', 'v' nodes and 'length'.
    """
    G = nx.Graph()
    for _, row in edges_gdf.iterrows():
        u = row['u']
        v = row['v']
        length = row['length']
        G.add_edge(u, v, weight=length)
    return G

def find_nearest_node(G, target_point):
    """
    Finds the nearest node in the graph to a given (lat, lon) point.
    Assumes node positions are stored as (lat, lon) in node attributes as 'coords'.
    """
    return min(G.nodes, key=lambda node: haversine(G.nodes[node]['coords'], target_point))

def dijkstra_route(G, source, target):
    """Compute the shortest path using Dijkstra's algorithm."""
    return nx.shortest_path(G, source=source, target=target, weight='weight')

def astar_route(G, source, target):
    """Compute the shortest path using A* algorithm."""
    return nx.astar_path(
        G,
        source=source,
        target=target,
        heuristic=lambda u, v: haversine(G.nodes[u]['coords'], G.nodes[v]['coords']),
        weight='weight'
    )

def dijkstra_algorithm(graph, start, goal):
    """
    Raw implementation of Dijkstra's algorithm.
    
    Args:
        graph: A NetworkX graph where edges have a 'weight' attribute.
        start: Starting node.
        goal: Goal node.
    
    Returns:
        A list of node IDs representing the shortest path.
    """
    visited = set()
    distances = {node: float('inf') for node in graph.nodes}
    previous = {}

    distances[start] = 0
    priority_queue = [(0, start)]

    while priority_queue:
        current_dist, current_node = heapq.heappop(priority_queue)

        if current_node in visited:
            continue
        visited.add(current_node)

        if current_node == goal:
            break

        for neighbor in graph.neighbors(current_node):
            weight = graph[current_node][neighbor]['weight']
            distance = current_dist + weight

            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous[neighbor] = current_node
                heapq.heappush(priority_queue, (distance, neighbor))

    # Reconstruct path
    path = []
    node = goal
    while node in previous:
        path.insert(0, node)
        node = previous[node]
    if path:
        path.insert(0, start)
    return path


def haversine(coord1, coord2):
    """Returns the geodesic distance in meters."""
    return geodesic(coord1, coord2).meters

def astar_algorithm(graph, start, goal):
    """
    Raw implementation of A* algorithm using Haversine distance as heuristic.
    
    Args:
        graph: A NetworkX graph with 'weight' on edges and 'coords' on nodes.
        start: Starting node.
        goal: Goal node.
    
    Returns:
        A list of node IDs representing the shortest path.
    """
    open_set = [(0, start)]
    came_from = {}

    g_score = {node: float('inf') for node in graph.nodes}
    f_score = {node: float('inf') for node in graph.nodes}

    g_score[start] = 0
    f_score[start] = haversine(graph.nodes[start]['coords'], graph.nodes[goal]['coords'])

    visited = set()

    while open_set:
        _, current = heapq.heappop(open_set)
        if current in visited:
            continue
        visited.add(current)

        if current == goal:
            break

        for neighbor in graph.neighbors(current):
            tentative_g = g_score[current] + graph[current][neighbor]['weight']
            if tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                heuristic = haversine(graph.nodes[neighbor]['coords'], graph.nodes[goal]['coords'])
                f_score[neighbor] = tentative_g + heuristic
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    # Reconstruct path
    path = []
    node = goal
    while node in came_from:
        path.insert(0, node)
        node = came_from[node]
    if path:
        path.insert(0, start)
    return path


def select_best_ambulance(graph, ambulances_df, incident_point, method='dijkstra'):
    """
    Given multiple ambulances and one incident point, select the most efficient ambulance
    based on shortest path cost using Dijkstra or A*.
    
    Returns:
        selected_ambulance (row from dataframe),
        best_path (list of node ids),
        best_cost (meters)
    """
    from algorithms import find_nearest_node, dijkstra_route, astar_route

    best_path = None
    best_cost = float('inf')
    selected_ambulance = None

    # Get nearest graph node to incident
    incident_node = find_nearest_node(graph, incident_point)

    for idx, row in ambulances_df.iterrows():
        amb_coords = (row['lat'], row['lon'])
        amb_node = find_nearest_node(graph, amb_coords)

        try:
            if method == 'dijkstra':
                path = dijkstra_route(graph, amb_node, incident_node)
            elif method == 'astar':
                path = astar_route(graph, amb_node, incident_node)
            else:
                raise ValueError("Invalid method")

            cost = sum(graph[u][v]['weight'] for u, v in zip(path[:-1], path[1:]))

            if cost < best_cost:
                best_cost = cost
                best_path = path
                selected_ambulance = row

        except Exception as e:
            print(f"Error routing from ambulance {idx}: {e}")

    return selected_ambulance, best_path, best_cost
