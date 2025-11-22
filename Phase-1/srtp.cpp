#include "graph.hpp"
#include "srtp.hpp"
 const double INF = std::numeric_limits<double>::infinity();
double calc_time(double curr_time,double length, std::vector<double>speed_profile)
{
    while(length>0)
    {
        int t = static_cast<int>(curr_time/15);
        double next_slot = ((floor(curr_time/15)+1)*15);
        double time_left = next_slot - curr_time;
        double distance_possible = speed_profile[t]*time_left;
        if(length <= distance_possible)
        {
            return curr_time + length/(double)speed_profile[t];
        }
        length-=distance_possible;
        curr_time = next_slot;
    }
    return curr_time;
}

std::vector<double> Graph::euclidean_distance_to(int target)  {
    const double R = 6371.0; // km
    int n=Nodes.size();
    std::vector<double> dist(n, std::numeric_limits<double>::infinity());
    if (target < 0 || target >= n) return dist;
    const Node* t = Nodes[target];
    if (!t) return dist;

    auto rad = [](double deg){ return deg * M_PI / 180.0; };
    double lat2 = rad(t->lat), lon2 = rad(t->lon);
    
    for (int i = 0; i < n; ++i) {
        const Node* ni = Nodes[i];
        if (!ni) continue;
        double lat1 = rad(ni->lat), lon1 = rad(ni->lon);
        double dlat = lat2 - lat1, dlon = lon2 - lon1;
        double a = std::sin(dlat*0.5); a = a*a;
        double b = std::sin(dlon*0.5); b = b*b;
        double h = a + std::cos(lat1)*std::cos(lat2)*b;
        dist[i] = 2.0 * R * std::atan2(std::sqrt(h), std::sqrt(std::max(0.0, 1.0-h)));
    }
    return dist;
}
double Graph::get_haversine_dist(int u_id, int v_id) {
    
    const double R = 6371000.0; 
    if (u_id < 0 || u_id >= Nodes.size() || v_id < 0 || v_id >= Nodes.size()) {
        return 1e9; 
    }
    const Node& node_u = *(Nodes[u_id]);
    const Node& node_v = *(Nodes[v_id]);
    auto to_rad = [](double degree) { return degree * M_PI / 180.0; };
    
    double lat1 = to_rad(node_u.lat);
    double lon1 = to_rad(node_u.lon);
    double lat2 = to_rad(node_v.lat);
    double lon2 = to_rad(node_v.lon);
    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;
    double a = std::sin(dlat / 2.0) * std::sin(dlat / 2.0) +
    std::cos(lat1) * std::cos(lat2) *
    std::sin(dlon / 2.0) * std::sin(dlon / 2.0);
    double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(std::max(0.0, 1.0 - a)));
    return R * c;
}

   std::vector<int> Graph::handleShortesPath(SRTP sp, bool &possible, double &mtbd) {
    if(sp.source>=Nodes.size() || sp.target >=Nodes.size() || sp.forbidden_nodes.count(sp.source) || sp.forbidden_nodes.count(sp.target))
    {
        return {};
    }
    if (sp.source == sp.target) {
        possible = true;
        mtbd = 0.0;
        return {sp.source};
    }

    
    if (adj_r.empty() || adj_r.size() != Nodes.size()) {
        // If adj_r wasn't built in constructor, we can't do bidirectional search
        possible = false; mtbd = -1.0; return {};
    }

    int n = Nodes.size();
    possible = false;
   
    double best_path_len = INF; // This is 'mu'
    int meet_node = -1;

    // 3. Priority Queues (Min-Heap)
    // Storing {distance, node_id}
    std::priority_queue<std::pair<double, int>, 
                        std::vector<std::pair<double, int>>, 
                        std::greater<std::pair<double, int>>> pq_f, pq_r;

    // 4. Data Structures
    // Initialize distances to Infinity
    std::vector<double> dist_f(n, INF);
    std::vector<double> dist_r(n, INF);
    // Parents for path reconstruction
    std::vector<int> parent_f(n, -1);
    std::vector<int> parent_r(n, -1);
    // Visited arrays (uint8_t is faster than vector<bool>)
    std::vector<uint8_t> vis_f(n, 0);
    std::vector<uint8_t> vis_r(n, 0);

    // 5. Initialization
    dist_f[sp.source] = 0.0;
    pq_f.push({0.0, sp.source});

    dist_r[sp.target] = 0.0;
    pq_r.push({0.0, sp.target});

    // 6. Main Loop
    while (!pq_f.empty() && !pq_r.empty()) {
        
        // --- Termination Condition ---
        // If the shortest potential connection (top of fwd + top of bwd) 
        // is worse than the best path we've already found, we can stop.
        double top_f = pq_f.top().first;
        double top_r = pq_r.top().first;
        if (top_f + top_r >= best_path_len) {
            break;
        }

        // --- Balanced Search Strategy ---
        // Always expand the smaller queue to keep search frontiers even
        if (pq_f.size() <= pq_r.size()) {
            // Expand Forward
            auto [d, u] = pq_f.top(); pq_f.pop();

            if (vis_f[u]) continue;
            vis_f[u] = 1;

            // Meeting Check (On Pop)
            if (vis_r[u]) {
                double new_len = dist_f[u] + dist_r[u];
                if (new_len < best_path_len) {
                    best_path_len = new_len;
                    meet_node = u;
                }
            }

            for (auto* e : adj[u]) {
                // Constraints
                if (sp.forbidden_nodes.count(e->v)) continue;
                if (sp.forbidden_road_types.count(e->road_type) && sp.forbidden_road_types.at(e->road_type)) continue;
                // Note: If you have forbidden edges, uncomment below:
                // if (sp.forbidden_edges.count({u, e->v})) continue;

                int v = e->v;
                double new_dist = dist_f[u] + e->length;

                if (new_dist < dist_f[v]) {
                    dist_f[v] = new_dist;
                    parent_f[v] = u;
                    pq_f.push({new_dist, v});

                    // Meeting Check (On Push - Optimization)
                    if (vis_r[v]) {
                        double potential_meet = new_dist + dist_r[v];
                        if (potential_meet < best_path_len) {
                            best_path_len = potential_meet;
                            meet_node = v;
                        }
                    }
                }
            }
        } else {
            // Expand Backward
            auto [d, u] = pq_r.top(); pq_r.pop();

            if (vis_r[u]) continue;
            vis_r[u] = 1;

            // Meeting Check (On Pop)
            if (vis_f[u]) {
                double new_len = dist_f[u] + dist_r[u];
                if (new_len < best_path_len) {
                    best_path_len = new_len;
                    meet_node = u;
                }
            }

            for (auto* e : adj_r[u]) {
                // Constraints (Check 'v' because in adj_r, 'v' is the node we came FROM physically)
                if (sp.forbidden_nodes.count(e->v)) continue;
                if (sp.forbidden_road_types.count(e->road_type) && sp.forbidden_road_types.at(e->road_type)) continue;
                // if (sp.forbidden_edges.count({e->v, u})) continue;

                int v = e->v;
                double new_dist = dist_r[u] + e->length;

                if (new_dist < dist_r[v]) {
                    dist_r[v] = new_dist;
                    parent_r[v] = u;
                    pq_r.push({new_dist, v});

                    // Meeting Check (On Push - Optimization)
                    if (vis_f[v]) {
                        double potential_meet = dist_r[v] + new_dist; // new_dist is dist_r[v] here
                        if (potential_meet < best_path_len) {
                            best_path_len = potential_meet;
                            meet_node = v;
                        }
                    }
                }
            }
        }
    }

    // 7. Path Reconstruction
    if (meet_node == -1 || best_path_len == INF) {
        possible = false;
        mtbd = INF;
        return {};
    }

    possible = true;
    mtbd = best_path_len;

    std::vector<int> path;
    // Trace Forward: Source -> Meet
    int curr = meet_node;
    while (curr != -1) {
        path.push_back(curr);
        curr = parent_f[curr];
    }
    std::reverse(path.begin(), path.end());

    // Trace Backward: Meet -> Target
    // Note: parent_r points "backwards" towards the target in the search tree
    curr = parent_r[meet_node];
    while (curr != -1) {
        path.push_back(curr);
        curr = parent_r[curr];
    }

    return path;
}


/////////////////////////////////////////////////A_star implementation
// std::vector<int> Graph::A_starimpl(SRTP sp, bool &possible, double & mtbd){
//     int n = Nodes.size();
//     possible = false; 
    
//     std::priority_queue<std::pair<double,int>,
//     std::vector<std::pair<double,int>>,
//     std::greater<std::pair<double,int>>> min_heap;
    
//     std::vector<double> dist(n, INF);
//     std::vector<int> parent(n, -1);
    
//     std::vector<double> h = euclidean_distance_to(sp.target);
    
//     dist[sp.source] = 0;
//     min_heap.push(std::make_pair(dist[sp.source] + h[sp.source], sp.source));
    
//     while(!min_heap.empty()){
//         auto [d, u] = min_heap.top();
//         min_heap.pop();
//         if (d > dist[u] + h[u]) continue;
//         if (u == sp.target) {
//             possible = true; 
//             break;
//         }
//         for(auto &e: adj[u]){
//             if(! (sp.forbidden_road_types[e->road_type]) && !sp.forbidden_nodes[e->v])
//             {
//                 int v = e->v;
//                 double length = e->length;
//                 if(dist[v] > dist[u] + length)
//                 {
//                     dist[v] = dist[u] + length;
//                     parent[v] = u;
//                     min_heap.push(std::make_pair(dist[v] + h[v], v));
//                 }
//              }
//         }
//     }
//     std::vector<int> path;
//     if(!possible) return path;
//     for(int i = sp.target; i != -1; i = parent[i])
//     {
//         path.push_back(i);
//     }
//     std::reverse(path.begin(), path.end());
//     return path;
// }
////////// to be tested;;;;;
std::vector<int> Graph::A_star_time(SRTP sp, bool &possible, double & mtbd){
      int n = Nodes.size();
    possible = false;
    std::priority_queue<std::pair<double,int>,
        std::vector<std::pair<double,int>>,
        std::greater<std::pair<double,int>>> min_heap;

    std::vector<double> arr(n, std::numeric_limits<double>::infinity());
    std::vector<int> parent(n, -1);

    if (sp.source < 0 || sp.source >= n || sp.target < 0 || sp.target >= n)
        return {};

    arr[sp.source] = 0.0;
    min_heap.push(std::make_pair(0.0, sp.source));

    while(!min_heap.empty()){
        auto [t, u] = min_heap.top();
        min_heap.pop();
        if (t > arr[u]) continue;

        if (u == sp.target) {
            possible = true;
            break;
        }

        for(auto &e: adj[u]){
            if (sp.forbidden_road_types.count(e->road_type) && sp.forbidden_road_types[e->road_type]) continue;
            if (sp.forbidden_nodes.count(e->v) && sp.forbidden_nodes[e->v]) continue;

            // calc_time returns absolute arrival time given current time
            double arrival = calc_time(arr[u], e->length, e->speed_profile);
            if (!std::isfinite(arrival)) continue;

            if (arrival < arr[e->v]) {
                arr[e->v] = arrival;
                parent[e->v] = u;
                min_heap.push(std::make_pair(arrival, e->v)); // push v not u
            }
        }
    }

    std::vector<int> path;
    if (!possible) return path;

    mtbd = arr[sp.target];
    for (int cur = sp.target; cur != -1; cur = parent[cur]) path.push_back(cur);
    std::reverse(path.begin(), path.end());
    return path;
}