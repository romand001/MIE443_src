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

// get the adjacent tiles of the given tile (8-connected neighbours)
std::vector<Map::Tile> Map::getAdjacent(Map::Tile s)
{
    // vector for storing the adjacent tiles
    std::vector<Map::Tile> adj;

    if (s.y != 0) {
        // add up tile
        Map::Tile u(s.x, s.y-1);
        u.occ = data_[u.x + u.y*width_];
        adj.push_back(u);
        // add up+left tile
        if (s.x != 0) {
            Map::Tile ul(s.x-1, s.y-1);
            ul.occ = data_[ul.x + ul.y*width_];
            adj.push_back(ul);
        }
        // add up+right tile
        if (s.x != width_-1) {
            Map::Tile ur(s.x+1, s.y-1);
            ur.occ = data_[ur.x + ur.y*width_];
            adj.push_back(ur);
        }
    }
    
    if (s.y != height_-1) {
        // add down tile
        Map::Tile d(s.x, s.y+1);
        d.occ = data_[d.x + d.y*width_];
        adj.push_back(d);
        // add down+left tile
        if (s.x != 0) {
            Map::Tile dl(s.x-1, s.y+1);
            dl.occ = data_[dl.x + dl.y*width_];
            adj.push_back(dl);
        }
        // add down+right tile
        if (s.x != width_-1) {
            Map::Tile dr(s.x+1, s.y+1);
            dr.occ = data_[dr.x + dr.y*width_];
            adj.push_back(dr);
        }
    }

    if (s.x != 0) {
        // add left tile
        Map::Tile l(s.x-1, s.y);
        l.occ = data_[l.x + l.y*width_];
        adj.push_back(l);
    }
    if (s.x != width_-1) {
        // add right tile
        Map::Tile r(s.x-1, s.y);
        r.occ = data_[r.x + r.y*width_];
        adj.push_back(r);
    }
}

// find the closest frontier to the given x and y coordinates
uint32_t* Map::closestFrontier(float x, float y) {

    //////////////////////////////////////////////////////
    //                  BFS Algorithm                   //
    //////////////////////////////////////////////////////

    // 2D vector of booleans to keep track of visited tiles
    std::vector<std::vector<bool>> visited(
        width_,
        std::vector<bool>(height_, false)
    );
    
    // queue to store tiles that need to be checked
    std::queue<Map::Tile> queue;

    // create starting tile, set its occupancy value, set it to visited, push it to the queue
    Map::Tile s((uint32_t)x, (uint32_t)y);
    s.occ = data_[s.x + s.y*width_];
    visited[x][y] = true;
    queue.push(s);

    bool foundFirst = false; 

    // process every tile in the queue or until a frontier is found
    while (queue.size() != 0 && !foundFirst) {
        // set s to the first queue item and then pop it from the queue
        s = queue.front();
        queue.pop();

        // get tiles adjacent to s tile
        std::vector<Map::Tile> adj = getAdjacent(s);
        
        // iterate over its adjacent tiles
        for (auto a: adj) {
            // if tile s is an open space and it has an adjacent unexplored tile
            // this means that s belongs to a frontier cluster
            if (s.occ == 0 && a.occ == -1) {
                foundFirst = true;
                break;
            }
            // if tile at these coords has not been visited, add to the queue and set to visited
            if (!visited[a.x][a.y]) {
                visited[a.x][a.y] = true;
                queue.push(a);
            }

        }

    }

    //////////////////////////////////////////////////////
    //              Border Detection (DFS)              //
    //////////////////////////////////////////////////////

    // store traversed tiles in a vector
    std::vector<Map::Tile> border;

    // vector for keeping track of visits efficiently
    // indexed the same as OccupancyGrid data
    std::vector<bool> visitedTiles(width_*height_, false);

    // stack for determining traversal order
    std::stack<Map::Tile> stack;

    // push starting frontier tile to stack
    stack.push(s);

    bool reachedEnd = false;
    // traverse non-visited frontier tiles until none are left
    // we begin at the frontier tile we found above (s)
    while (!stack.empty()) {

        // grab current tile from stack and pop it
        s = stack.top();
        stack.pop();

        border.push_back(s);

        // get adjacent tiles
        std::vector<Map::Tile> adj = getAdjacent(s);

        bool hasNeighbours = false;
        // iterate over adjacent tiles
        for (auto a: adj) {
            // if adjacent is a frontier and has not been visited
            if (s.occ == 0 && a.occ == -1 && 
                    !visitedTiles[a.x+a.y*width_]) {
                // set visited, push to stack
                visitedTiles[a.x+a.y*width_] = true;
                stack.push(a);
                hasNeighbours = true;
                break;
            }
        }
        if (!hasNeighbours) break;

    }

    //////////////////////////////////////////////////////
    //                  Cluster Center                  //
    //////////////////////////////////////////////////////
    uint64_t xSum = 0; uint64_t ySum = 0;
    for (auto b: border) {
        xSum += b.x;
        ySum += b.y;
    }

    // return pointer to coordinate array of cluster
    static uint32_t coords[2] = {(uint32_t)(xSum/border.size()), (uint32_t)(ySum/border.size())};
    return coords;


}

// scans entire map and locates frontiers (BFS is probably better than this)
std::vector<std::vector<bool>> Map::frontierScan()
{
    // create map of frontiers
    std::vector<std::vector<bool>> frontierMap(
        width_,
        std::vector<bool>(height_, false)
    );

    // iterate over tiles (width and height)
    for (int x=0; x<width_; x++) {
        for (int y=0; y<height_; y++) {

            // create Tile object
            Map::Tile tile((uint32_t)x, (uint32_t)y);
            // set tile occupancy
            tile.occ = data_[tile.x + tile.y*width_];

            // check if tile is empty space
            if (tile.occ == 0) {
                // get adjacent tiles
                std::vector<Map::Tile> adj = getAdjacent(tile);

                // iterate over the adjacent tiles
                for (auto a: adj) {
                    // check if adjacent tile is unexplored, then tile is a frontier
                    if (a.occ == -1) {
                        frontierMap[x][y] = true;
                        break;
                    }

                }

            }

            

        }
    }
    return frontierMap;
}

} // namespace end