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

        // std::cout << "Parsed OK\n";
        // std::cout << "Nodes: " << g.getNodeCount() << "\n";

        // // optional: print sample node/edge info using existing helper
        // g.printSampleData();

        // // lightweight validation: check speed_profile lengths and non-negative values
        // bool ok = true;
        // for(size_t u = 0; u < g.getNodeCount() && u < 10; ++u){ /* only sample few nodes */ }
        // std::cout << (ok ? "Basic validation passed\n" : "Basic validation failed\n");
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