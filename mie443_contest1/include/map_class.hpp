#include "ros/ros.h"
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include <map>
#include <queue>

namespace mainSpace {

class Map {

    struct Tile {
        uint32_t x;
        uint32_t y;
        uint32_t occ;
        Tile(uint32_t xt, uint32_t yt);
    };

private:
    uint32_t width_;
    uint32_t height_;
    std::vector<int8_t> data_;
    std::vector<std::vector<int8_t>> grid_;

public:
    Map(uint32_t width, uint32_t height, std::vector<int8_t> data);
    ~Map() = default;
    // void populate(const std::vector<int8> data);
    void info();
    void print();
    // int8_t getOccupancy(uint32_t x, uint32_t y);
    std::vector<Tile> getAdjacent(Tile t);
    void BFS(float x, float y);

};

} // namespace end