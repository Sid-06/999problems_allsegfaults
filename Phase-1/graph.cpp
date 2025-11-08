#include"graph.hpp"
#include<fstream>
#include<algorithm>

Graph::Graph(const json& graph_json){
    if(graph_json.contains("nodes")){
        for(auto& nd : graph_json["nodes"]){
            Node* node = new Node(nd["id"],nd["lat"],nd["lon"],nd["pois"]);
            Nodes.push_back(node);
            
        }
    }
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
                edge->speed_profile.assign(96, 50.0);
            }

            // bounds check for u/v
            if (u >= 0 && u < N) adj[u].push_back(edge);
            if (!oneway && v >= 0 && v < N) adj[v].push_back(edge);
            // keep reverse adjacency if you need it elsewhere
            if (v >= 0 && v < N) adj_r[v].push_back(edge);
            if (!oneway && u >= 0 && u < N) adj_r[u].push_back(edge);
        }
    }

};
json Graph::handleRemoveEdge(const json& query) {
    json res;
    int eid = query["id"];
    Edge* target = nullptr;
    for (auto& p : adj) {
        for (auto e : p) {
            if (e->id == eid) { target = e; break; }
        }
        if (target) break;
    }
    if (!target) {
        res["error"] = "edge not found";
        return res;
    }
    auto &vec_u = adj[target->u];
    vec_u.erase(std::remove(vec_u.begin(), vec_u.end(), target), vec_u.end());

    auto &vec_v = adj[target->v];
    vec_v.erase(std::remove(vec_v.begin(), vec_v.end(), target), vec_v.end());
    delete target;
    res["removed"] = true;
    res["edge_id"] = eid;
    return res;
};

json Graph::handleModifyEdge(const json& query) {
    json res;
    int eid = query["id"];
    Edge* target = nullptr;

    for (auto& p : adj) {
        for (auto e : p) {
            if (e->id == eid) { target = e; break; }
        }
        if (target) break;
    }
    if (!target) {
        res["error"] = "edge not found";
        return res;
    }
    if (query.contains("length")) target->length = query["length"];
    if (query.contains("avg_time")) target->avg_time = query["avg_time"];
    if (query.contains("oneway")) target->oneway = query["oneway"];
    if (query.contains("road_type")) target->road_type = query["road_type"];
    res["modified"] = true;
    res["edge_id"] = eid;
    return res;
};


json Graph::process_query(const json& query) {
    //  SRTP sp;
    // sp.source = query["source"];
    // sp.target = query["target"];
    
    // // Initialize forbidden nodes map
    // for (const auto& fn : query["forbidden_nodes"]) {
    //     sp.forbidden_nodes[fn.get<int>()] = true;
    // }
    json res;
    if (!query.contains("type")){
        res["error"] = "missing type";
        return res;
    }
    std::string t = query["type"];
    if (t == "remove_edge"){
        res = handleRemoveEdge(query);
    }
    else if (t == "modify_edge"){
        res = handleModifyEdge(query);
    }
    else if (t == "shortest_path"){
        SRTP sp;
        sp.id = query["id"];
        sp.source = query["source"];
        sp.target = query["target"];
        sp.mode = query["mode"];
        if (query.contains("forbidden_nodes")) {
            for (const auto& fn : query["forbidden_nodes"]) sp.forbidden_nodes.insert(fn);
        }
        if (query.contains("forbidden_road_types")) {
            for (auto it = query["forbidden_road_types"].begin(); it != query["forbidden_road_types"].end(); ++it) {
                sp.forbidden_road_types[it.key()] = true;
            }
        }
        double mtbd = 0.0;
        bool possible = false;
        std::vector<int> path = handleShortesPath(sp, possible, mtbd);
        res["possible"] = possible;
        res["mtbd"] = mtbd;
        res["path"] = path;
    } else if (t == "knn"){
        KNN kn;
        kn.id = query["id"];
        kn.poi = query["poi"];
        kn.lat = query["query_point"]["lat"];
        kn.lon = query["query_point"]["lon"];
        kn.k = query["k"];
        kn.metric = query["metric"];

        std::vector<int> ans = handleKnn(kn);
        res["result"] = ans;
    } else {
        res["error"] = "unsupported query type: " + t;
    }
    return res;
}

// #include"graph.hpp"
// #include<fstream>

// Graph::Graph(const json& graph_json){
//     if(graph_json.contains("nodes")){
//         for(auto& nd : graph_json["nodes"]){
//             Node* node = new Node(nd["id"],nd["lat"],nd["lon"],nd["pois"]);
//             Nodes.push_back(node);
            
//         }
//     }
//     N = Nodes.size();
//     adj.resize(N);
//     adj_r.resize(N);

//     if(graph_json.contains("edges")){
//         for (const auto& ed : graph_json["edges"]) {
//             Edge* edge = new Edge(
//                 ed["id"].get<int>(),
//                 ed["u"].get<int>(),
//                 ed["v"].get<int>(),
//                 ed["length"].get<double>(),
//                 ed["avg_time"].get<double>(),
//                 ed["speed_profile"],
//                 ed["oneway"].get<bool>(),
//                 ed["road_type"].get<std::string>()
//             );

//             // add into adjacency vectors
//             adj[edge->u].push_back(edge);
//             if (!edge->oneway) {
//                 adj[edge->v].push_back(edge);
//             }
//         }
//     }

// };
// json Graph::handleRemoveEdge(const json& query) {
//     json res;
//     int eid = query["id"];
//     Edge* target = nullptr;
//     for (auto& p : adj) {
//         for (auto e : p) {
//             if (e->id == eid) { target = e; break; }
//         }
//         if (target) break;
//     }
//     if (!target) {
//         res["error"] = "edge not found";
//         return res;
//     }
//    auto &vec_u = adj[target->u];
//     vec_u.erase(std::remove(vec_u.begin(), vec_u.end(), target), vec_u.end());

//     auto &vec_v = adj[target->v];
//     vec_v.erase(std::remove(vec_v.begin(), vec_v.end(), target), vec_v.end());
//     delete target;
//     res["removed"] = true;
//     res["edge_id"] = eid;
//     return res;
// };

// json Graph::handleModifyEdge(const json& query) {
//     json res;
//     int eid = query["id"];
//     Edge* target = nullptr;

//     for (auto& p : adj) {
//         for (auto e : p) {
//             if (e->id == eid) { target = e; break; }
//         }
//         if (target) break;
//     }
//     if (!target) {
//         res["error"] = "edge not found";
//         return res;
//     }
//     if (query.contains("length")) target->length = query["length"];
//     if (query.contains("avg_time")) target->avg_time = query["avg_time"];
//     if (query.contains("oneway")) target->oneway = query["oneway"];
//     if (query.contains("road_type")) target->road_type = query["road_type"];
//     res["modified"] = true;
//     res["edge_id"] = eid;
//     return res;
// };


// json Graph::process_query(const json& query) {
//     json res;
//     if (!query.contains("type")){
//         res["error"] = "missing type";
//         return res;
//     }
//     std::string t = query["type"];
//     if (t == "remove_edge"){
//         res = handleRemoveEdge(query);
//     }
//     else if (t == "modify_edge"){
//         res = handleModifyEdge(query);
//     }
//     else if (t == "shortest_path"){
//         SRTP sp;
//         sp.id = query["id"];
//         sp.source = query["source"];
//         sp.target = query["target"];
//         sp.mode = query["mode"];
//         if (query.contains("forbidden_nodes")) {
//             for (const auto& fn : query["forbidden_nodes"]) sp.forbidden_nodes.insert(fn);
//         }
//         if (query.contains("forbidden_road_types")) {
//             for (auto it = query["forbidden_road_types"].begin(); it != query["forbidden_road_types"].end(); ++it) {
//                 sp.forbidden_road_types[it.key()] = true;
//             }
//         }
//         double mtbd = 0.0;
//         bool possible = false;
//         std::vector<int> path = handleShortesPath(sp, possible, mtbd);
//         res["possible"] = possible;
//         res["mtbd"] = mtbd;
//         res["path"] = path;
//     } else if (t == "knn"){
//         KNN kn;
//         kn.id = query["id"];
//         kn.poi = query["poi"];
//         kn.lat = query["query_point"]["lat"];
//         kn.lon = query["query_point"]["lon"];
//         kn.k = query["k"];
//         kn.metric = query["metric"];

//         std::vector<int> ans = handleKnn(kn);
//         res["result"] = ans;
//     } else {
//         res["error"] = "unsupported query type: " + t;
//     }
//     return res;
// }
