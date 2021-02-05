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
    nav_msgs::MapMetaData map_info_;
    uint32_t width_;
    uint32_t height_;
    std::vector<int8_t> data_;
    // 2D vector of mappings between grid Tiles and pointers to all adjacent grid Tiles
    std::vector<std::vector<AdjacencyRelationship>> adjacencyGrid_;

public:
    Map();
    Map(nav_msgs::MapMetaData map_info);
    //~Map();
    void info();
    void update(std::vector<int8_t> data);
    std::map<float, float> closestFrontier(float xf, float yf);
    nav_msgs::MapMetaData getInfo();
    uint32_t getWidth();
    uint32_t getHeight();
    std::pair<float, float> mapToPos(uint32_t intX, uint32_t intY);
    std::pair<uint32_t, uint32_t> posToMap(float floatX, float floatY);

    // deprecated
    void print();
    // deprecated
    std::vector<Tile> getAdjacent(Tile s);
    // deprecated
    std::vector<std::vector<bool>> frontierScan();

};

} // namespace end