#include "map_class.hpp"

namespace mainSpace {

// constructor for the Tile struct, sets the coord and occupancy
Map::Tile::Tile(uint32_t xt, uint32_t yt) 
    :x(xt), 
    y(yt) {};

Map::AdjacencyRelationship::AdjacencyRelationship(Map::Tile selft, std::vector<Map::Tile> adjt)
    :self(selft),
    adjacentTiles(adjt) {}

// Map class constructor, define adjacency relationships for graph traversal
Map::Map()
{
    //ROS_INFO("map object created with w: %u, h: %u", width_, height_);

    // iterate over tiles to fill adjacency grid
    for (int y=0; y < HEIGHT; y++) {
        std::vector<Map::AdjacencyRelationship> adjacencyRow;
        for (int x=0; x<WIDTH; x++) {
            // declare vector for storing adjacent tiles
            std::vector<Map::Tile> adjacentTiles;
            
            // fill adjacent tile vector
            if (y != 0) {
                // up tile
                Map::Tile u(x, y-1);
                u.occ = data_.data() + x + WIDTH*(y-1);
                adjacentTiles.push_back(u);
                if (x != 0) {
                    // up+left tile
                    Map::Tile ul(x-1, y-1);
                    ul.occ = data_.data() + x-1 + WIDTH*(y-1);
                    adjacentTiles.push_back(ul);
                }
                if (x != WIDTH-1) {
                    // up+right tile
                    Map::Tile ur(x+1, y-1);
                    ur.occ = data_.data() + x+1 + WIDTH*(y-1);
                    adjacentTiles.push_back(ur);
                }
            }
            if (y != HEIGHT-1) {
                // down tile
                Map::Tile d(x, y+1);
                d.occ = data_.data() + x + WIDTH*(y+1);
                adjacentTiles.push_back(d);
                if (x != 0) {
                    // down+left tile
                    Map::Tile dl(x-1, y+1);
                    dl.occ = data_.data() + x-1 + WIDTH*(y+1);
                    adjacentTiles.push_back(dl);
                }
                if (x != WIDTH-1) {
                    // down+right tile
                    Map::Tile dr(x+1, y+1);
                    dr.occ = data_.data() + x+1 + WIDTH*(y+1);
                    adjacentTiles.push_back(dr);
                }
            }

            if (x != 0) {
                // left tile
                Map::Tile l(x-1, y);
                l.occ = data_.data() + x-1 + WIDTH*(y);
                adjacentTiles.push_back(l);
            }
            if (x != WIDTH-1) {
                // right tile
                Map::Tile r(x+1, y);
                r.occ = data_.data() + x+1 + WIDTH*(y);
                adjacentTiles.push_back(r);
            }

            // create an adjacency relationship object with the relationships found
            Map::Tile t(x, y);
            Map::AdjacencyRelationship rel(t, adjacentTiles);
            // push the relationship to the adjacency grid
            adjacencyRow.push_back(rel);

        }
        // push the current row to the grid
        adjacencyGrid_.push_back(adjacencyRow);
    }

}

// iterate over occupancy grid and print how many of each number occurs
// useful for figuring out what's going on without printing thousands of numbers
// note: std::map has nothing to do with the robot map, it is just a data structure 
void Map::info()
{
    // map (dictionary-like data structure) of <data number: how many times it occurs>
    std::map<int8_t, uint32_t> gridVals;
    // iterate through data and increment the value that it matches in gridVals
    for (int i = 0; i < WIDTH * HEIGHT; i++) {
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
void Map::update(std::vector<int8_t> data) {data_ = data;}

// print each number in the occupancy grid
// you probably should not call this function
/*
void Map::print()
{
    for (int i=0; i<width_*height_; i++) {
        std::cout << data_[i] << " ";
        if (i % width_ == 0) std::cout << std::endl;
    }
}
*/

// get the adjacent tiles of the given tile (8-connected neighbours)
// DEPRECATED
/*
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
*/

// find the closest frontier to the given x and y coordinates
uint32_t* Map::closestFrontier(float xf, float yf) {

    //////////////////////////////////////////////////////
    //                  BFS Algorithm                   //
    //////////////////////////////////////////////////////

    // 2D vector of booleans to keep track of visited tiles
    std::vector<std::vector<bool>> visited(
        WIDTH,
        std::vector<bool>(HEIGHT, false)
    );
    
    // queue to store adjacency relationships (tiles and their adjacencies)
    std::queue<Map::AdjacencyRelationship> queue;

    //current x and y coordinates
    uint32_t x = (uint32_t)xf; uint32_t y = (uint32_t)yf; 

    // set starting tile to visited, push it to the queue
    Map::AdjacencyRelationship rel = adjacencyGrid_[x][y];
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
            else if (!visited[a.x][a.y]) {
                // set this tile's position as visited
                visited[a.x][a.y] = true;
                // push the adjacency relationship to the queue and loop back
                queue.push(adjacencyGrid_[a.x][a.y]);
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
    // different name than above (visited), but same purpose
    std::vector<bool> visitedTiles(WIDTH*HEIGHT, false);

    // stack of adjacency relationships for determining traversal order
    std::stack<Map::AdjacencyRelationship> stack;

    // push starting frontier relationship to stack (perhaps edge of frontier)
    stack.push(rel);

    bool hasNeighbours = false;
    // traverse non-visited frontier tiles until none are left
    // we begin at the frontier tile we found above (tile of rel)
    while (!stack.empty() && !hasNeighbours) {

        // grab current relationship from stack and pop it
        rel = stack.top();
        stack.pop();

        // store tile in border vector
        border.push_back(rel.self);

        // get adjacent tiles
        std::vector<Map::Tile> adj = rel.adjacentTiles;

        hasNeighbours = false;
        // iterate over adjacent tiles
        for (auto a: adj) {
            // if adjacent is a frontier and has not been visited
            if (*rel.self.occ == 0 && *a.occ == -1 && 
                    !visitedTiles[a.x + WIDTH*a.y]) {
                // set visited, push relationship to stack
                visitedTiles[a.x + WIDTH*a.y] = true;
                stack.push(adjacencyGrid_[a.x][a.y]);
                hasNeighbours = true;
                break;
            }
        }

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
// DEPRECATED
/*
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
*/

} // namespace end