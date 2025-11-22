import requests
import json
import math
import random

# --- CONFIGURATION ---
# Bounding Box (South, West, North, East) - Example: Part of Mumbai
BBOX = [19.0700, 72.8700, 19.0800, 72.8800]

# Strict constraints
ALLOWED_ROAD_TYPES = ["primary", "secondary", "tertiary", "local", "expressway"]
ALLOWED_POIS = ["restaurant", "petrol station", "hospital", "pharmacy", "hotel", "atm"]
POI_PROBABILITY = 0.2
QUERY_COUNT = 20  # How many queries to generate

# Mapping OSM types to your strict types
OSM_MAPPING = {
    "motorway": "expressway", "trunk": "expressway",
    "primary": "primary", "secondary": "secondary", "tertiary": "tertiary",
    "unclassified": "local", "residential": "local", "service": "local"
}
SPEED_OPTS = {"expressway": 100, "primary": 60, "secondary": 50, "tertiary": 40, "local": 30}

def haversine_meters(lat1, lon1, lat2, lon2):
    R = 6371000.0
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat / 2)**2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

def fetch_and_process_graph(bbox):
    # 1. Query Overpass
    query = f"""
    [out:json];
    (way["highway"]({bbox[0]},{bbox[1]},{bbox[2]},{bbox[3]}); node(w););
    out body; >; out skel qt;
    """
    print("Fetching map data...")
    try:
        resp = requests.get("http://overpass-api.de/api/interpreter", params={'data': query})
        data = resp.json()
    except:
        print("API Request failed. Check internet or API status.")
        return None

    # 2. Process Nodes
    osm_nodes = {}
    for el in data['elements']:
        if el['type'] == 'node':
            osm_nodes[el['id']] = {"lat": el['lat'], "lon": el['lon']}

    # 3. Map to 0-based IDs and Assign POIs
    node_map = {}
    final_nodes = []
    current_id = 0
    
    # Only map nodes used in ways
    used_ids = set()
    ways = [el for el in data['elements'] if el['type'] == 'way']
    for w in ways:
        for n in w['nodes']: used_ids.add(n)

    for oid in used_ids:
        if oid in osm_nodes:
            node_map[oid] = current_id
            pois = [random.choice(ALLOWED_POIS)] if random.random() < POI_PROBABILITY else []
            final_nodes.append({
                "id": current_id,
                "lat": osm_nodes[oid]['lat'],
                "lon": osm_nodes[oid]['lon'],
                "pois": pois
            })
            current_id += 1

    # 4. Process Edges
    final_edges = []
    edge_counter = 0
    seen_pairs = set()

    for w in ways:
        raw_type = w.get('tags', {}).get('highway', 'local')
        rtype = OSM_MAPPING.get(raw_type, "local")
        oneway = w.get('tags', {}).get('oneway', 'no') == 'yes'
        speed_ms = SPEED_OPTS[rtype] / 3.6

        nodes = w['nodes']
        for i in range(len(nodes)-1):
            u_osm, v_osm = nodes[i], nodes[i+1]
            if u_osm not in node_map or v_osm not in node_map: continue
            
            u, v = node_map[u_osm], node_map[v_osm]
            
            # Geometry
            n1, n2 = osm_nodes[u_osm], osm_nodes[v_osm]
            dist = haversine_meters(n1['lat'], n1['lon'], n2['lat'], n2['lon'])
            if dist < 0.1: dist = 0.1
            time = dist / speed_ms
            
            # 96-slot profile
            profile = [speed_ms * random.uniform(0.8, 1.0) for _ in range(96)]

            def add_e(src, dst):
                nonlocal edge_counter
                if (src, dst) in seen_pairs: return
                final_edges.append({
                    "id": edge_counter, "u": src, "v": dst,
                    "length": round(dist, 3), "avg_time": round(time, 3),
                    "road_type": rtype, "speed_profile": profile,
                    "oneway": oneway # kept for consistency, though graph is simple
                })
                seen_pairs.add((src, dst))
                edge_counter += 1

            add_e(u, v)
            if not oneway: add_e(v, u)

    return {"nodes": final_nodes, "edges": final_edges}

def generate_queries(graph, count):
    queries = []
    nodes = graph['nodes']
    edges = graph['edges']
    
    if not nodes or not edges: return []

    for i in range(1, count + 1):
        qtype = random.choice(["shortest_path", "modify_edge", "remove_edge", "knn"])
        
        q = {"id": i}
        
        if qtype == "shortest_path":
            src = random.choice(nodes)['id']
            tgt = random.choice(nodes)['id']
            while src == tgt: tgt = random.choice(nodes)['id']
            
            q["type"] = "shortest_path"
            q["source"] = src
            q["target"] = tgt
            q["mode"] = random.choice(["time", "distance"])
            
            # Add constraints randomly
            if random.random() < 0.3:
                q["constraints"] = {}
                if random.random() < 0.5:
                    # Forbidden Node (pick a random 3rd node)
                    fn = random.choice(nodes)['id']
                    q["constraints"]["forbidden_nodes"] = [fn]
                else:
                    # Forbidden Type
                    q["constraints"]["forbidden_road_types"] = [random.choice(ALLOWED_ROAD_TYPES)]

        elif qtype == "modify_edge":
            edge = random.choice(edges)
            q["type"] = "modify_edge"
            q["edge_id"] = edge['id']
            # Patch object as requested
            q["patch"] = {}
            if random.random() < 0.5:
                q["patch"]["length"] = round(edge['length'] * 2.0, 2)
            else:
                q["patch"]["road_type"] = "local"

        elif qtype == "remove_edge":
            edge = random.choice(edges)
            q["type"] = "remove_edge"
            q["edge_id"] = edge['id']

        elif qtype == "knn":
            q["type"] = "knn"
            q["poi"] = random.choice(ALLOWED_POIS)
            # Pick a random spot inside the BBOX
            lat = random.uniform(BBOX[0], BBOX[2])
            lon = random.uniform(BBOX[1], BBOX[3])
            q["query_point"] = {"lat": round(lat, 6), "lon": round(lon, 6)}
            q["k"] = random.randint(1, 5)
            q["metric"] = random.choice(["shortest_path","euclidean"])

        queries.append(q)
    
    return {"meta": {"comment": "Generated Test Cases"}, "events": queries}

def main():
    graph_data = fetch_and_process_graph(BBOX)
    if not graph_data: return

    # Write graph.json
    with open("graph.json", "w") as f:
        json.dump(graph_data, f, indent=4)
    print(f"graph.json created ({len(graph_data['nodes'])} nodes, {len(graph_data['edges'])} edges)")

    # Generate and Write queries.json
    queries_data = generate_queries(graph_data, QUERY_COUNT)
    with open("queries.json", "w") as f:
        json.dump(queries_data, f, indent=4)
    print(f"queries.json created ({len(queries_data['events'])} queries)")

if __name__ == "__main__":
    main()