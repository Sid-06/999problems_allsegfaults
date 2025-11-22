#include"graph.hpp"
#include<fstream>
#include<algorithm>

Graph::Graph(const json& graph_json){
    if(graph_json.contains("nodes") && graph_json["nodes"].is_array()){
        for(auto& nd : graph_json["nodes"]){
            Node* node = new Node(nd["id"],nd["lat"],nd["lon"],nd["pois"]);
            Nodes.push_back(node);
            
        }
    }
    current_token = 0;
    N = Nodes.size();
    adj.clear();
    adj.resize(N);
    adj_r.clear();
    adj_r.resize(N);
    if (graph_json.contains("edges") && graph_json["edges"].is_array()) {
        for (const auto &ed : graph_json["edges"]) {
            int id = ed.value("id", -1);
            int u  = ed.value("u", -1);
            int v  = ed.value("v", -1);
            double length = ed.value("length", 0.0);
            double avg_time = ed.value("avg_time", 0.0);
            bool oneway = ed.value("oneway", true);
            std::string road_type = ed.value("road_type", std::string());

            // create Edge*
            Edge* edge = new Edge();
            edge->id = id;
            edge->u = u;
            edge->v = v;
            edge->length = length;
            edge->avg_time = avg_time;
            edge->oneway = oneway;
            edge->road_type = road_type;

            // speed_profile: copy if present, otherwise fill with default 96 slots
            edge->speed_profile.clear();
            if (ed.contains("speed_profile") && ed["speed_profile"].is_array()) {
                for (const auto &s : ed["speed_profile"]) {
                    edge->speed_profile.push_back(s.get<double>());
                }
            }
            if (edge->speed_profile.empty()) {
                edge->speed_profile.assign(96, (edge->length/edge->avg_time));
            }
            Edges.push_back(edge);
            // bounds check for u/v
            if (u >= 0 && u < N) adj[u].push_back(edge);
            if (!oneway && v >= 0 && v < N) adj[v].push_back(edge);
            // keep reverse adjacency if you need it elsewhere
            if (v >= 0 && v < N) adj_r[v].push_back(edge);
            if (!oneway && u >= 0 && u < N) adj_r[u].push_back(edge);
        }
    }
    edge_visited_token.assign(Edges.size()+1,0);
};
json Graph::handleRemoveEdge(const json& query) {
    json res;
    int eid = query["edge_id"];
    Edge* target = nullptr;
    for (auto& p : Edges) {
            if (p->id == eid) { target = p; break; }
        }
    if (!target) {
        res["error"] = "edge not found";
        res["removed"] = false;
        return res;
    }
    if(target->active == false) res["removed"] = false;
    else res["removed"] = true;
    res["edge_id"] = eid;
    return res;
};

json Graph::handleModifyEdge(const json& query) {
    json res;
    if (query.contains("id")) {
        res["id"] = query["id"];
    }

    int eid = query["edge_id"];
    const json& patch = query["patch"];

    Edge* target = nullptr;
    for (auto& p : Edges) {
        if (p->id == eid) { target = p; break; }
    }

    if (!target) {
        res["done"] = false;
        return res;
    }

    if (target->active) {
        if (patch.empty()) {
            res["done"] = false;
            return res;
        }
    } else {
        target->active = true;
    }

    if (patch.contains("length")) target->length = patch["length"];
    if (patch.contains("average_time")) target->avg_time = patch["average_time"];
    if (patch.contains("speed_profile")) target->speed_profile = patch["speed_profile"].get<std::vector<double>>();
    if (patch.contains("road_type")) target->road_type = patch["road_type"];

    res["done"] = true;
    return res;
}

json Graph::process_query(const json& query) {
    json res;
    if (!query.contains("type")){
        res["error"] = "missing type";
        return res;
    }
    std::string t = query["type"];
    if (t == "K_shortest_paths"){
        SRTP sp;
        sp.id = query["id"];
        sp.source = query["source"];
        sp.target = query["target"];
        if(query.contains("mode")) sp.mode = query["mode"];
        if (query.contains("constraints")){
        if (query["constraints"].contains("forbidden_nodes")) {
            for (const auto& fn : query["constraints"]["forbidden_nodes"]) sp.forbidden_nodes[fn] = true;
        }
        if (query["constraints"].contains("forbidden_road_types")) {
            if (query["constraints"].contains("forbidden_road_types") && query["constraints"]["forbidden_road_types"].is_array()) {
            for (const auto& val : query["constraints"]["forbidden_road_types"]) {
              sp.forbidden_road_types[val] = true;
                    }
                }   
            }
        }
        std::list<std::pair<double,std::vector<int>>> paths = handle_K_ShortestPaths(sp,query["k"]);
        res["paths"] = json::array();
        for (const auto& entry : paths){
            json j_entry;
            j_entry["path"] = entry.second;
            j_entry["length"] = entry.first;
            res["paths"].push_back(j_entry);
        }
    }else if(t == "k_shortest_paths_heuristic"){
        KSP_H ksp;
        ksp.id = query["id"];
        ksp.source = query["source"];
        ksp.target = query["target"];
        ksp.k = query["k"];
        ksp.overlap_threshold = query["overlap_threshold"];
        std::vector<std::pair<std::vector<int>,double>> paths = k_shortest_paths_heustirics(ksp);
        res["paths"] = json::array();
        for (const auto& entry : paths){
            json j_entry;
            j_entry["path"] = entry.first;
            j_entry["length"] = entry.second;
            res["paths"].push_back(j_entry);
        }
    }else if(t == "approx_shortest_path"){
        double epsilon = query["acceptable_error_pct"];
        double timeout = query["time_budget_ms"];
        for(auto vals : query["queries"]){
            json j_entry;
            j_entry["source"] = vals["source"];
            j_entry["target"] = vals["target"];
            j_entry["approx_shortest_distance"] = weighted_a_star(vals["source"], vals["target"],epsilon,timeout); 
            res["distances"].push_back(j_entry);
        }
    }else {
            res["error"] = "unsupported query type: " + t;
    }
    return res;
}
