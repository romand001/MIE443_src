#include "map_class.hpp"

namespace mainSpace {

Map::Tile::Tile(uint32_t xt, uint32_t yt) : x(xt), y(yt) {}

// constructor, just set width and height
Map::Map(const uint32_t width, const uint32_t height, const std::vector<int8_t> data)
    :width_(width),
    height_(height),
    data_(data)
{
    // put other constructor stuff in here
    ROS_INFO("map object created with w: %u, h: %u", width_, height_);
}

// fill the map with data from OccupancyGrid
// void Map::populate(const std::vector<int8> data)
// {
//     for (int x=0; x<width_; x++) {
//         std::vector<int8_t> col;
//         for (int y=0; y<height_; y++) {
//             col.push_back(data[x + y*width_]);
//         }
//         grid_.push_back(col);
//     }
// }

// iterate over occupancy grid and print how many of each number occurs
void Map::info()
{
    std::map<int8_t, uint32_t> gridVals;
    for (int i = 0; i < width_ * height_; i++) {
        gridVals[data_[i]]++;
    }
    std::map<int8_t, uint32_t>::iterator it;
    for (it = gridVals.begin(); it != gridVals.end(); it++) {
        std::cout << (int32_t)it->first << ": " << it->second << std::endl;
    }
}

// print each number in the occupancy grid
// you probably should not call this function
void Map::print()
{
    for (int y=0; y<height_; y++) {
        for (int x=0; x<width_; x++) {
            std::cout << grid_[x][y] << " ";
        }
        std::cout << std::endl;
    }
}

// // get occupancy value at specified coordinates
// // called by Tile constructor
// int8_t Map::getOccupancy(uint32_t x, uint32_t y)
// {
//     return data_[x + y*width_];
// }

// get the adjacent tiles of the given tile
std::vector<Map::Tile> Map::getAdjacent(Map::Tile s)
{

    std::vector<Map::Tile> adj;

    // get up tile
    if (s.y != 0) {
        Map::Tile u(s.x, s.y-1);
        u.occ = data_[u.x + u.y*width_];
        adj.push_back(u);
    }
    // get down tile
    if (s.y != height_-1) {
        Map::Tile d(s.x, s.y+1);
        d.occ = data_[d.x + d.y*width_];
        adj.push_back(d);
    }
    // get left tile
    if (s.x != 0) {
        Map::Tile l(s.x-1, s.y);
        l.occ = data_[l.x + l.y*width_];
        adj.push_back(l);
    }
    // get right tile
    if (s.x != width_-1) {
        Map::Tile r(s.x-1, s.y);
        r.occ = data_[r.x + r.y*width_];
        adj.push_back(r);
    }
}

// perform Breadth-First-Search starting at x and y (looking for closest frontier)
void Map::BFS(float x, float y)
{
    // 2D vector of booleans to keep track of visited tiles
    std::vector<std::vector<bool>> visited(
        width_,
        std::vector<bool>(height_, false)
    );
    
    // queue to store tiles that need to be checked
    std::queue<Map::Tile> queue;

    Map::Tile s((uint32_t)x, (uint32_t)y);
    s.occ = data_[s.x + s.y*width_];
    visited[x][y] = true;
    queue.push(s);

    bool foundNearest = false;

    while (queue.size() != 0 && !foundNearest) {
        s = queue.front();
        queue.pop();

        // check here if Tile s is a frontier, set foundNearest=true


        std::vector<Map::Tile> adj;
        for (auto t: adj) {
            if (!visited[t.x][t.y]) {
                visited[t.x][t.y] = true;
                queue.push(t);
            }

        }

    }



}

} // namespace end