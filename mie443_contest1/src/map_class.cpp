#include "map_class.hpp"

namespace mainSpace {

// constructor for the Tile struct, sets the coord and occupancy
Map::Tile::Tile(uint32_t xt, uint32_t yt) 
    :x(xt), 
    y(yt) {};

Map::AdjacencyRelationship::AdjacencyRelationship(Map::Tile selft, std::vector<Map::Tile> adjt)
    :self(selft),
    adjacentTiles(adjt) {}

// empty constructor for initializing map without map_info
Map::Map() {}

// Map class constructor, define adjacency relationships for graph traversal
Map::Map(nav_msgs::MapMetaData map_info) 
    :map_info_(map_info),
    width_(map_info_.width),
    height_(map_info_.height)
{
    ROS_INFO("map object created with w: %u, h: %u", width_, height_);

    data_.resize(width_*height_, 0);

    // iterate over tiles to fill adjacency grid
    for (int x=0; x < width_; x++) {
        std::vector<Map::AdjacencyRelationship> adjacencyCol;
        for (int y=0; y<height_; y++) {
            // declare vector for storing adjacent tiles
            std::vector<Map::Tile> adjacentTiles;
            
            // fill adjacent tile vector
            if (y != 0) {
                // up tile
                Map::Tile u(x, y-1);
                u.occ = &data_[0] + x + width_*(y-1);
                adjacentTiles.push_back(u);
                if (x != 0) {
                    // up+left tile
                    Map::Tile ul(x-1, y-1);
                    ul.occ = &data_[0] + x-1 + width_*(y-1);
                    adjacentTiles.push_back(ul);
                }
                if (x != width_-1) {
                    // up+right tile
                    Map::Tile ur(x+1, y-1);
                    ur.occ = &data_[0] + x+1 + width_*(y-1);
                    adjacentTiles.push_back(ur);
                }
            }
            if (y != height_-1) {
                // down tile
                Map::Tile d(x, y+1);
                d.occ = &data_[0] + x + width_*(y+1);
                adjacentTiles.push_back(d);
                if (x != 0) {
                    // down+left tile
                    Map::Tile dl(x-1, y+1);
                    dl.occ = &data_[0] + x-1 + width_*(y+1);
                    adjacentTiles.push_back(dl);
                }
                if (x != width_-1) {
                    // down+right tile
                    Map::Tile dr(x+1, y+1);
                    dr.occ = &data_[0] + x+1 + width_*(y+1);
                    adjacentTiles.push_back(dr);
                }
            }

            if (x != 0) {
                // left tile
                Map::Tile l(x-1, y);
                l.occ = &data_[0] + x-1 + width_*(y);
                adjacentTiles.push_back(l);
            }
            if (x != width_-1) {
                // right tile
                Map::Tile r(x+1, y);
                r.occ = &data_[0] + x+1 + width_*(y);
                adjacentTiles.push_back(r);
            }

            // create an adjacency relationship object with the relationships found
            Map::Tile t(x, y);
            t.occ = &data_[0] + x + width_*y;
            Map::AdjacencyRelationship rel(t, adjacentTiles);
            // push the relationship to the adjacency grid
            adjacencyCol.push_back(rel);

        }
        // push the current row to the grid
        adjacencyGrid_.push_back(adjacencyCol);
    }
    ROS_INFO("finished creating adjacency relationships");
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

// update map data
// adjacency grid is also updated because it points to data_
void Map::update(std::vector<int8_t> data) {
    //ROS_INFO("before data update");
    data_ = data;
    //ROS_INFO("after data update");
    }

// find the closest frontier to the given x and y coordinates
std::map<float, float> Map::closestFrontier(float xf, float yf) {
    

    //ROS_INFO("entered closest frontier algorithm");

    //////////////////////////////////////////////////////
    //                  BFS Algorithm                   //
    //////////////////////////////////////////////////////

    // 2D vector of booleans to keep track of visited tiles
    std::vector<std::vector<bool>> visited(
        width_,
        std::vector<bool>(height_, false)
    );

    std::pair<uint32_t, uint32_t> mapCoords = posToMap(xf, yf);
    uint32_t x = mapCoords.first;
    uint32_t y = mapCoords.second;
    
    // queue to store adjacency relationships (tiles and their adjacencies)
    std::queue<Map::AdjacencyRelationship> queue;

    ROS_INFO("robot float coordinates are x=%f, y=%f", xf, yf);

    if (x>adjacencyGrid_.size() || y>adjacencyGrid_[0].size()) {
        ROS_ERROR("BFS starting point of x=%u, y=%u is out of bounds in grid of dims x=%lu, y=%lu",
                    x, y, adjacencyGrid_.size(), adjacencyGrid_[0].size());
        std::map<float, float> emptyMap;
        return emptyMap;
    }

    Map::AdjacencyRelationship rel = adjacencyGrid_[x][y];

    // set starting tile to visited, push it to the queue
    visited[x][y] = true;
    queue.push(rel);

    bool foundFirst = false; 

    // process every tile in the queue or until a frontier is found
    while (queue.size() != 0 && !foundFirst) {
        // set rel to the first queue item and then pop it from the queue
        rel = queue.front();
        queue.pop();

        // get the adjacent tiles to this tile
        std::vector<Map::Tile> adj = rel.adjacentTiles;
        
        // iterate over its adjacent tiles
        for (auto a: adj) {
            // if the tile in rel is an open space and it has an adjacent unexplored tile
            // this means that the rel tile belongs to a frontier cluster
            if (*rel.self.occ == 0 && *a.occ == -1) {
                foundFirst = true;
            }
            // if tile at these coords has not been visited, add to the queue and set to visited
            // tile must also be open space in order to be added, so that search does not go through walls
            else if (!visited[a.x][a.y] && *a.occ == 0) {
                // set this tile's position as visited
                visited[a.x][a.y] = true;
                // push the adjacency relationship to the queue and loop back

                // ROS_INFO("tried pushing adjacencyGrid_ of size %u by %u at indices [%u][%u] to queue",
                //             adjacencyGrid_.size(), adjacencyGrid_[0].size(), a.x, a.y);
                queue.push(adjacencyGrid_[a.x][a.y]);
            }

        }

    }

    ROS_INFO("finished BFS, found first frontier at x=%u, y=%u", rel.self.x, rel.self.y);
    //ROS_INFO("this frontier tile has %lu neighbours", rel.adjacentTiles.size());

    //////////////////////////////////////////////////////
    //              Border Detection (DFS)              //
    //////////////////////////////////////////////////////

    // store traversed tiles in a vector
    std::vector<Map::Tile> border;

    // vector for keeping track of visits efficiently
    // indexed the same as OccupancyGrid data
    // different name than above (visited), but same purpose
    std::vector<bool> visitedTiles(width_*height_, false);

    // stack of adjacency relationships for determining traversal order
    std::stack<Map::AdjacencyRelationship> stack;

    // push starting frontier relationship to stack (perhaps edge of frontier)
    stack.push(rel);

    // traverse non-visited frontier tiles until none are left
    // we begin at the frontier tile we found above (tile of rel)
    while (!stack.empty()) {

        // grab current relationship from stack and pop it
        rel = stack.top();
        stack.pop();

        // store tile in border vector
        border.push_back(rel.self);

        // get adjacent tiles
        std::vector<Map::Tile> adj = rel.adjacentTiles;

        // iterate over adjacent tiles
        for (auto a: adj) {
            // if this neighour is an open space and hasn't been visited
            if (*a.occ == 0 && !visitedTiles[a.x + width_*a.y]) {
                // set to visited
                visitedTiles[a.x + width_*a.y] = true;
                // get tiles adjacent to the neighbour and iterate over them
                Map::AdjacencyRelationship rel2 = adjacencyGrid_[a.x][a.y];
                std::vector<Map::Tile> adj2 = rel2.adjacentTiles;
                for (auto a2: adj2) {
                    // if any of the adjacent tile's neighbours is an unknown space, add the tile to stack
                    if (*a2.occ == -1) {
                        stack.push(rel2);
                        break;
                    }
                }
            }
            
        }

    }

    ROS_INFO("traversed border and found %lu tiles", border.size());

    // return the coordinates of each tile on the frontier border
    std::map<float, float> frontierCoords;

    for (auto frontier: border) {
            frontierCoords.insert(mapToPos(frontier.x, frontier.y));
    }
    // uncomment this to plot robot coords as well:
    // frontierCoords.insert(std::pair<float, float>(xf, yf));
    return frontierCoords;

}

nav_msgs::MapMetaData Map::getInfo()
{
    return map_info_;
}

uint32_t Map::getWidth() 
{
    return width_;
}

uint32_t Map::getHeight() 
{
    return height_;
}

std::pair<float, float> Map::mapToPos(uint32_t intX, uint32_t intY)
{
    float xCoord = ((float)intX * map_info_.resolution) + map_info_.origin.position.x;
    float yCoord = ((float)intY * map_info_.resolution) + map_info_.origin.position.y;
    std::pair<float, float> floatPair(xCoord, yCoord);
    return floatPair;
}

std::pair<uint32_t, uint32_t> Map::posToMap(float floatX, float floatY)
{
    uint32_t x = 0;
    uint32_t y = 0;

    float xt = (floatX - map_info_.origin.position.x) / map_info_.resolution;
    if (xt >= 0) x = (uint32_t)ceil(xt);
    else ROS_WARN("xt=%f, will use 0 instead", xt);
    float yt = (floatY - map_info_.origin.position.y) / map_info_.resolution;
    if (yt >= 0) y = (uint32_t)ceil(yt);
    else ROS_WARN("yt=%f, will use 0 instead", yt);

    std::pair<uint32_t, uint32_t> intPair(x, y);
    return intPair;
}

// scans entire map and locates frontiers (BFS is probably better than this)
// DEPRECATED
/*
std::vector<std::vector<bool>> Map::frontierScan()
{
    // create map of frontiers
    std::vector<std::vector<bool>> frontierMap(
        width__,
        std::vector<bool>(height__, false)
    );

    // iterate over tiles (width_ and height_)
    for (int x=0; x<width__; x++) {
        for (int y=0; y<height__; y++) {

            // create Tile object
            Map::Tile tile((uint32_t)x, (uint32_t)y);
            // set tile occupancy
            tile.occ = data_[tile.x + tile.y*width__];

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
*/

} // namespace end