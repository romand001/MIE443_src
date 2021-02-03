#include "ros/ros.h"
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include <map>
#include <queue>
#include <stack>

#define WIDTH 256
#define HEIGHT 256

namespace mainSpace {

class Map {

    struct Tile {
        uint32_t x;
        uint32_t y;
        int8_t* occ;
        Tile(uint32_t xt, uint32_t yt);
    };

    struct AdjacencyRelationship {
        Tile self;
        std::vector<Tile> adjacentTiles;
        AdjacencyRelationship(Tile selft, std::vector<Tile> adjt);
    };

private:
    // width_ and height_ are deprecated variables, use defines now
    // uint32_t width_;
    // uint32_t height_;
    std::vector<int8_t> data_;
    // 2D vector of mappings between grid values and pointers to all adjacent grid values
    std::vector<std::vector<AdjacencyRelationship>> adjacencyGrid_;

public:
    Map();
    //~Map();
    void info();
    void update(std::vector<int8_t> data);
    uint32_t* closestFrontier(float x, float y);

    // deprecated
    void print();
    // deprecated
    std::vector<Tile> getAdjacent(Tile s);
    // deprecated
    std::vector<std::vector<bool>> frontierScan();

};

} // namespace end