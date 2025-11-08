#ifndef KDTREE_H
#define KDTREE_H

#include "../common/graph.hpp"
#include <vector>
#include <string>
#include <set>


struct kdnode{
    int id;
    std::set<std::string> poi;
    double lat;
    double lon;
    kdnode* left = nullptr;
    kdnode* right = nullptr;

    kdnode(int id_, double lat_, double lon_, const std::set<std::string>& poi_) : id(id_), poi(poi_), lat(lat_), lon(lon_), left(nullptr), right(nullptr) {}
    
};

class kdtree
{
private:

    kdnode* root;
   

    // Helpers
    kdnode* buildRecursive(std::vector<kdnode*>& points, int depth);
    kdnode* insertRecursive(kdnode* root, kdnode* node, int depth);
    void clear(kdnode* node);

public:
    kdtree();                           
    ~kdtree();                         

    kdnode* buildtree(const Graph graph); // Builds the KD-tree

    void add(const kdnode& node); // adds a new node

    bool empty(); // check is empty

    kdnode* getRoot(); // return root

    std::vector<kdnode*> knn(const kdnode* root, const kdnode& target, int k) ;
};

#endif 

