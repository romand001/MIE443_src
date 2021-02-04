#include "ros/ros.h"
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include <map>
#include <queue>
#include <stack>

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
    uint32_t width_;
    uint32_t height_;
    std::vector<int8_t> data_;
    // 2D vector of mappings between grid values and pointers to all adjacent grid values
    std::vector<std::vector<AdjacencyRelationship>> adjacencyGrid_;

public:
    Map(uint32_t width, uint32_t height);
    //~Map();
    void info();
    void update(std::vector<int8_t> data);
    std::pair<uint32_t, uint32_t> closestFrontier(float x, float y);
    uint32_t getWidth();
    uint32_t getHeight();

    // deprecated
    void print();
    // deprecated
    std::vector<Tile> getAdjacent(Tile s);
    // deprecated
    std::vector<std::vector<bool>> frontierScan();

};

} // namespace end