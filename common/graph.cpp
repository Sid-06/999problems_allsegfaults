#include"graph.hpp"
#include<fstream>

Graph::Graph(const json& graph_json){
    if(graph_json.contains("nodes")){
        for(auto& nd : graph_json["nodes"]){
            Node* node = new Node(nd["id"],nd["lat"],nd["lon"],nd["pois"]);
            Nodes.push_back(node);
            
        }
    }
    N = Nodes.size();
    if(graph_json.contains("edges")){
        for(auto& ed : graph_json["edges"]){
            Edge* edge = new Edge(ed["id"],ed["u"],ed["v"],ed["length"],ed["avg_time"],ed["speed_profile"],ed["oneway"],ed["road_type"]);
            if(ed["oneway"]) adj[ed["u"]].insert(edge);
            else{
                adj[ed["u"]].insert(edge);
                adj[ed["v"]].insert(edge);
            }
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
    adj[target->u].erase(target);
    adj[target->v].erase(target);
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
            for (const auto& fn : query["forbidden_nodes"]) sp.forbidden_nodes.push_back(fn);
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
