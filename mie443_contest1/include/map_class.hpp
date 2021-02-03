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
    void info();
    void print();
    std::vector<Tile> getAdjacent(Tile s);
    uint32_t* closestFrontier(float x, float y);
    std::vector<std::vector<bool>> frontierScan();

};

} // namespace end