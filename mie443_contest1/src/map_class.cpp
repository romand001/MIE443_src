#include "map_class.hpp"

namespace mainSpace {

// constructor for the Tile struct, sets the x and y values
Map::Tile::Tile(uint32_t xt, uint32_t yt) : x(xt), y(yt) {}

// Map class constructor, just set width, height, and data
Map::Map(const uint32_t width, const uint32_t height, const std::vector<int8_t> data)
    :width_(width),
    height_(height),
    data_(data)
{
    // put other constructor stuff in here
    ROS_INFO("map object created with w: %u, h: %u", width_, height_);
}

// iterate over occupancy grid and print how many of each number occurs
// useful for figuring out what's going on without printing thousands of numbers
// note: std::map has nothing to do with the robot map, it is just a data structure 
void Map::info()
{
    // map (dictionary-like data structure) of <data number: how many times it occurs>
    std::map<int8_t, uint32_t> gridVals;
    // iterate through data and increment the value that it matches in gridVals
    for (int i = 0; i < width_ * height_; i++) {
        gridVals[data_[i]]++;
    }
    // iterate through the map data structure and print out with nice formatting
    std::map<int8_t, uint32_t>::iterator it;
    for (it = gridVals.begin(); it != gridVals.end(); it++) {
        std::cout << (int32_t)it->first << ": " << it->second << std::endl;
    }
}

// print each number in the occupancy grid
// you probably should not call this function
void Map::print()
{
    for (int i=0; i<width_*height_; i++) {
        std::cout << data_[i] << " ";
        if (i % width_ == 0) std::cout << std::endl;
    }
}

// get the adjacent tiles of the given tile
std::vector<Map::Tile> Map::getAdjacent(Map::Tile s)
{
    // vector for storing the adjacent tiles
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

    // create starting tile, set its value, set it to visited, push it to the queue
    Map::Tile s((uint32_t)x, (uint32_t)y);
    s.occ = data_[s.x + s.y*width_];
    visited[x][y] = true;
    queue.push(s);

    // will set this to true when the first frontier is found, exits the while loop
    bool foundNearest = false;

    // process every tile in the queue (will keep going until not tiles left or we exit)
    while (queue.size() != 0 && !foundNearest) {
        // set s to the first queue item and then pop it from the queue
        s = queue.front();
        queue.pop();

        // check here if Tile s is a frontier, set foundNearest=true to exit
        //
        //

        // get adjacent tiles of s and iterate over them
        std::vector<Map::Tile> adj = getAdjacent(s);
        for (auto t: adj) {
            // if tile at these coords has not been visited, add to the queue and set to visited
            if (!visited[t.x][t.y]) {
                visited[t.x][t.y] = true;
                queue.push(t);
            }

        }

    }



}

} // namespace end