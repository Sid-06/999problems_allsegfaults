#include "graph.hpp"
#include "srtp.hpp"
#include "knn.h"
#include <fstream>
#include <iostream>

int main(int argc, char** argv){
    const char* json_path = "converted_graph.json";
    if(argc > 1) json_path = argv[1];

    std::ifstream ifs(json_path);
    if(!ifs){ std::cerr << "failed to open " << json_path << "\n"; return 2; }

    try{
        json graph_json = json::parse(ifs);
        Graph g(graph_json);
        // SRTP sp;
        // sp.id = 1;
        // sp.source = 0;
        // sp.target = 100;
        // sp.mode = "distance";
        //  double mtbd = 0.0;
        // bool possible = false;
        KNN kn;
        kn.id = 1;
        kn.poi = {""};
        kn.lat = 18.491497;
        kn.lon = 83.3510416;
        kn.k =  18;
        kn.metric = {"shortest_path"};

        auto start_time = std::chrono::high_resolution_clock::now();
        std::vector<int> ans = g.handleKnn(kn);
        auto end_time = std::chrono::high_resolution_clock::now();
        std::cout<<std::chrono::duration<double, std::milli>(end_time - start_time).count();
        for(auto&v:ans){
            std::cout<<v <<" ";
        }
           }
    catch(const json::parse_error &e){
        std::cerr << "JSON parse error: " << e.what() << "\n";
        return 1;
    }
    catch(const std::exception &e){
        std::cerr << "Exception: " << e.what() << "\n";
        return 1;
    }
    return 0;
}