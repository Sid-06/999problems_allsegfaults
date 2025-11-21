// #include "nlohmann/json.hpp"
// #include "nlohmann/json_fwd.hpp"
// #include <iostream>
// #include <fstream>
// #include <chrono>
// #include "graph.hpp"
// /*
//     Add other includes that you require, only write code wherever indicated
// */

// using json = nlohmann::json;

// int main(int argc, char* argv[]) {
//     if (argc != 3) {
//         std::cerr << "Usage: " << argv[0] << " <graph.json> <queries.json>" << std::endl;
//         return 1;
//     }

//     // Read graph from first file
//     std::ifstream graph_file(argv[1]);
//     if (!graph_file.is_open()) {
//         std::cerr << "Failed to open " << argv[1] << std::endl;
//         return 1;
//     }
//     json graph_json;
//     graph_file >> graph_json;
//     graph_file.close();

//     Graph graph(graph_json);
//     /*
//         Add your graph reading and processing code here
//         Initialize any classes and data structures needed for query processing
//     */

//     // Read queries from second file
//     std::ifstream queries_file(argv[2]);
//     if (!queries_file.is_open()) {
//         std::cerr << "Failed to open " << argv[2] << std::endl;
//         return 1;
//     }
//     json queries_json;
//     queries_file >> queries_json;

//     std::ofstream output_file("output.json");
//     if (!output_file.is_open()) {
//         std::cerr << "Failed to open output.json for writing" << std::endl;
//         return 1;
//     }

//     for (const auto& query : queries_json) {
//         auto start_time = std::chrono::high_resolution_clock::now();

//         /*
//             Add your query processing code here
//             Each query should return a json object which should be printed to sample.json
//         */

//         // Answer each query replacing the function process_query using 
//         // whatever function or class methods that you have implemented
//         json result = graph.process_query(query);

//         auto end_time = std::chrono::high_resolution_clock::now();
//         result["processing_time"] = std::chrono::duration<double, std::milli>(end_time - start_time).count();

//         output_file << result.dump(4) << '\n';
//     }

//     output_file.close();
//     return 0;
// }
//////////////////////////////////////////////////////////

#include "nlohmann/json.hpp"
#include "nlohmann/json_fwd.hpp"
#include <iostream>
#include <fstream>
#include <chrono>
#include <vector>
#include "graph.hpp"

using json = nlohmann::json;

int main(int argc, char* argv[]) {
    // Specification says: ./phaseX <graph.json> <queries.json> <output.json>
    // This results in argc = 4
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <graph.json> <queries.json> <output.json>" << std::endl;
        return 1;
    }

    // 1. Read graph from first file
    std::ifstream graph_file(argv[1]);
    if (!graph_file.is_open()) {
        std::cerr << "Failed to open graph file: " << argv[1] << std::endl;
        return 1;
    }
    json graph_json;
    try {
        graph_file >> graph_json;
    } catch (const json::parse_error& e) {
        std::cerr << "JSON parse error in graph file: " << e.what() << std::endl;
        return 1;
    }
    graph_file.close();

    // Initialize Graph
    // Ensure your Graph constructor handles the specific logic for the current Phase
    Graph graph(graph_json);

    // 2. Read queries from second file
    std::ifstream queries_file(argv[2]);
    if (!queries_file.is_open()) {
        std::cerr << "Failed to open queries file: " << argv[2] << std::endl;
        return 1;
    }
    json queries_input;
    try {
        queries_file >> queries_input;
    } catch (const json::parse_error& e) {
        std::cerr << "JSON parse error in queries file: " << e.what() << std::endl;
        return 1;
    }
    queries_file.close();

    // 3. Prepare Output Structure
    json output_json;
    
    // Copy "meta" from input to output as per spec
    if (queries_input.contains("meta")) {
        output_json["meta"] = queries_input["meta"];
    }
    
    // Initialize results array
    output_json["results"] = json::array();

    // Open output file stream
    std::ofstream output_file(argv[3]);
    if (!output_file.is_open()) {
        std::cerr << "Failed to open output file for writing: " << argv[3] << std::endl;
        return 1;
    }

    // 4. Process Queries
    // The spec says queries are inside an "events" array in the input JSON
    if (queries_input.contains("events") && queries_input["events"].is_array()) {
        
        for (const auto& query : queries_input["events"]) {
            auto start_time = std::chrono::high_resolution_clock::now();
            json result;

            // Requirement: Ensure each query is enclosed within a try-catch block
            try {
                result = graph.process_query(query);
            } catch (const std::exception& e) {
                // Handle graceful failure without crashing the driver
                result = {
                    {"id", query.value("id", -1)}, // Try to preserve ID
                    {"error", e.what()},
                    {"status", "failed"}
                };
                std::cerr << "Error processing query ID " << query.value("id", -1) << ": " << e.what() << std::endl;
            } catch (...) {
                result = {
                    {"id", query.value("id", -1)},
                    {"error", "Unknown exception occurred"},
                    {"status", "failed"}
                };
                std::cerr << "Unknown error processing query ID " << query.value("id", -1) << std::endl;
            }

            auto end_time = std::chrono::high_resolution_clock::now();
            
            // Add processing time in milliseconds
            // Note: Spec mentions "processing_time": 12 (which implies integer ms or float)
            // Using double for precision, cast to int if strict integer required
            double duration_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
            result["processing_time"] = duration_ms;

            // Add to results array
            output_json["results"].push_back(result);
        }
    } else {
        std::cerr << "Warning: Input JSON does not contain valid 'events' array." << std::endl;
    }

    // 5. Write final JSON object to file
    // dump(4) provides pretty printing with 4-space indentation
    output_file << output_json.dump(4);
    output_file.close();

    return 0;
}