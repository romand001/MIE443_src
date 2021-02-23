#include "map_class.hpp"

namespace mainSpace {

// constructor for the Tile struct, sets the coord and occupancy
Map::Tile::Tile(uint32_t xt, uint32_t yt) 
    :x(xt), 
    y(yt) {}

Map::AdjacencyRelationship::AdjacencyRelationship(Map::Tile selft, std::vector<Map::Tile> adjt)
    :self(selft),
    adjacentTiles(adjt) {}

// constructor without parent (for start)
Map::Tile_Info::Tile_Info(uint32_t xt, uint32_t yt, 
                          uint32_t end_x, uint32_t end_y, int8_t occ)
    :x(xt),
    y(yt)
{
    pathLength = weightFactor * occ;
    endDist = sqrt(pow(end_x - x, 2) + pow(end_y - y, 2));
    totalCost = pathLength + endDist;
    checked = false;
}

// constructor with parent
Map::Tile_Info::Tile_Info(uint32_t xt, uint32_t yt, uint32_t end_x, uint32_t end_y, 
                          Tile_Info* parentT, int8_t occ)
    :x(xt),
    y(yt),
    parent(parentT)
{
    pathLength = parent->pathLength
                + sqrt(abs(x - parent->x) + abs(y - parent->y))
                + weightFactor * occ;
    endDist = sqrt(pow(end_x - x, 2) + pow(end_y - y, 2));
    totalCost = pathLength + endDist;
    checked = false;
}

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
    data_smoothed_.resize(width_*height_, 0);

    // iterate over tiles to fill adjacency grid
    createAdjacencyRelationship_(data_, &adjacencyGrid_);
    ROS_INFO("finished creating adjacency relationship for base map");
    createAdjacencyRelationship_(data_smoothed_, &smoothedAdjacencyGrid_);
    ROS_INFO("finished creating adjacency relationships for smoothed map");
}


// create a adjacency relationship out of the data vector, and store it in the vector pointed to by adjacencyGrid
void Map::createAdjacencyRelationship_(std::vector<int8_t> data,
                                       std::vector<std::vector<AdjacencyRelationship>> *adjacencyGrid) {
    for (int x=0; x < width_; x++) {
        std::vector<Map::AdjacencyRelationship> adjacencyCol;
        for (int y=0; y<height_; y++) {
            // declare vector for storing adjacent tiles
            std::vector<Map::Tile> adjacentTiles;

            // fill adjacent tile vector
            if (y != 0) {
                // up tile
                Map::Tile u(x, y-1);
                u.occ = &data[0] + x + width_ * (y - 1);
                adjacentTiles.push_back(u);
                if (x != 0) {
                    // up+left tile
                    Map::Tile ul(x-1, y-1);
                    ul.occ = &data[0] + x - 1 + width_ * (y - 1);
                    adjacentTiles.push_back(ul);
                }
                if (x != width_-1) {
                    // up+right tile
                    Map::Tile ur(x+1, y-1);
                    ur.occ = &data[0] + x + 1 + width_ * (y - 1);
                    adjacentTiles.push_back(ur);
                }
            }
            if (y != height_-1) {
                // down tile
                Map::Tile d(x, y+1);
                d.occ = &data[0] + x + width_ * (y + 1);
                adjacentTiles.push_back(d);
                if (x != 0) {
                    // down+left tile
                    Map::Tile dl(x-1, y+1);
                    dl.occ = &data[0] + x - 1 + width_ * (y + 1);
                    adjacentTiles.push_back(dl);
                }
                if (x != width_-1) {
                    // down+right tile
                    Map::Tile dr(x+1, y+1);
                    dr.occ = &data[0] + x + 1 + width_ * (y + 1);
                    adjacentTiles.push_back(dr);
                }
            }

            if (x != 0) {
                // left tile
                Map::Tile l(x-1, y);
                l.occ = &data[0] + x - 1 + width_ * (y);
                adjacentTiles.push_back(l);
            }
            if (x != width_-1) {
                // right tile
                Map::Tile r(x+1, y);
                r.occ = &data[0] + x + 1 + width_ * (y);
                adjacentTiles.push_back(r);
            }

            // create an adjacency relationship object with the relationships found
            Map::Tile t(x, y);
            t.occ = &data[0] + x + width_ * y;
            Map::AdjacencyRelationship rel(t, adjacentTiles);
            // push the relationship to the adjacency grid
            adjacencyCol.push_back(rel);

        }
        // push the current row to the grid
        adjacencyGrid->push_back(adjacencyCol);
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
    data_ = data;

    // *****add bumper data to data_
    // *****cycle through data_ and make invis coords equal 100

    for (int i=0; i < invis.size(); i++) {
        int xBump = invis[i].first;
        int yBump = invis[i].second;
        data_smoothed_[xBump + width_*(yBump)] = 100; 
    }
    
    updateDilated(2);
    ROS_INFO("finished creating smoothed map");
}


// create and return dilated map based on unsmoothed_data, where radius is the size of the dilation (robot size), in map units
std::vector<int8_t> Map::generateDilated_(uint32_t radius, std::vector<int8_t> unsmoothed_data) {
    if (radius < 1) // dilation complete
        return unsmoothed_data;

    std::vector<int8_t> new_data(unsmoothed_data.size());
    for (int i = 0; i < data_.size(); i++) {
        if (unsmoothed_data[i] == 100) { // 100 means grid is filled
            new_data[i] = 100; // fill centre
            // fill the squares to one unit in each direction
            if (i % width_ > 0) // col > 0 (i.e., col - 1 >= 0)
                new_data[i - 1] = 100; // left
            if (i % width_ + 1 < width_) // col + 1 < width
                new_data[i + 1] = 100; // right
            if (i - width_ > 0) // row - 1 > 0
                new_data[i - width_] = 100; // up
            if (i + width_ < new_data.size()) // row + 1 < height
                new_data[i + width_] = 100; // down
        } else if (new_data[i] != 100) { // fill original unknowns and free spaces
            new_data[i] = unsmoothed_data[i];
        }
    }
    return generateDilated_(radius - 1, new_data); // repeat for 1 smaller radius
}


// pad the data to add extra rows and columns (actually not currently used)
std::vector<int8_t> Map::padData_(uint32_t amount, std::vector<int8_t> data) {
    std::vector<int8_t> new_data = data;
    std::vector<int8_t> row1 = {data.begin(), data.begin() + width_};
    std::vector<int8_t> rowl = {data.end() - width_, data.end()};
    for (int i = 0; i < amount; i++) {
        new_data.insert(new_data.begin(), row1.begin(), row1.end());
        new_data.insert(new_data.end(), rowl.begin(), rowl.end());
    }

    data = new_data;
    uint32_t new_width = width_ + 2 * amount;
    for (int j = 0; j < height_ + 2 * amount; j++) {
        for (int i = 0; i < amount; i++) {
            new_data.insert(new_data.begin() + new_width * j, data[width_ * j]);
            new_data.insert(new_data.begin() + new_width * (j + i + 1), data[width_ * j]);
        }
    }

    return new_data;
}


// smooth the adjacency map using a mean filter; note that kernel_size must be *odd*
std::vector<int8_t> Map::generateSmoothed_(uint32_t kernel_size, std::vector<int8_t> unsmoothed_data) {
    int32_t padding = (kernel_size - 1) / 2; // amount that kernel extends past centre pixel on each side
    std::vector<int32_t> horizontal_sums(unsmoothed_data.size()); // same size/grid meaning as data_; holds sum of map pixels (that are not -1 for unknown) within kernel range horizontally
    std::vector<uint8_t> horizontal_sum_counts(unsmoothed_data.size()); // number of values added to the sum at this point
    uint32_t curr_sum; // running sum
    uint32_t sum_count; // running number of values in sum

    for (int i = 0; i < height_; i++) { // iterate rows
        curr_sum = 0;
        sum_count = 0;

        // starting sum for first element in row; contains sum over half kernel window including and to the right of the first element, except the last element because it will be added in the first iteration of the nested loop
        for (int k = 0; k < padding; k++) {
            if (unsmoothed_data[width_ * i + k] >= 0) {
                curr_sum += unsmoothed_data[width_ * i + k];
                sum_count++;
            }
        }

        for (int j = 0; j < width_; j++) { // iterate columns
            int32_t curr_ind = width_ * i + j;

            // adding to running sum to the right if that cell is within bounds and not -1
            if (j + padding < width_ && unsmoothed_data[curr_ind + padding] >= 0) {
                curr_sum += unsmoothed_data[curr_ind + padding];
                sum_count++;
            }

            // subtracting from running sum to the left if that cell is within bounds and not -1
            if (j - padding - 1 >= 0 && unsmoothed_data[curr_ind - padding - 1] >= 0) {
                curr_sum -= unsmoothed_data[curr_ind - padding - 1];
                sum_count--;
            }

            // only update for cells that are currently not -1 to keep unknowns unknown
            if (unsmoothed_data[curr_ind] >= 0)
                horizontal_sums[curr_ind] = curr_sum;
            else
                horizontal_sums[curr_ind] = unsmoothed_data[curr_ind]; // keep -1

            horizontal_sum_counts[curr_ind] = sum_count;
        }
    }

    // repeat the process, but iterating first over columns and then rows, with appropriate indexing adjustments
    std::vector<int32_t> vertical_sums(unsmoothed_data.size()); // sum of all values within 2D window of kernel size centred on each cell
    std::vector<uint8_t> vertical_sum_counts(unsmoothed_data.size()); // corresponding counts

    for (int i = 0; i < width_; i++) {
        curr_sum = 0;
        sum_count = 0;
        for (int k = 0; k < padding; k++) {
            if (unsmoothed_data[width_ * k + i] >= 0) {
                curr_sum += horizontal_sums[width_ * k + i];
                sum_count++;
            }
        }
        for (int j = 0; j < height_; j++) {
            int32_t curr_ind = width_ * j + i;

            if (j + padding < height_ && unsmoothed_data[curr_ind + padding * width_] >= 0) {
                curr_sum += horizontal_sums[curr_ind + padding * width_];
                sum_count += horizontal_sum_counts[curr_ind + padding * width_];
            }
            if (j - padding - 1 >= 0 && unsmoothed_data[curr_ind - (padding + 1) * width_] >= 0) {
                curr_sum -= horizontal_sums[curr_ind - (padding + 1) * width_];
                sum_count -= horizontal_sum_counts[curr_ind - (padding + 1) * width_];
            }
            if (unsmoothed_data[curr_ind] >= 0)
                vertical_sums[curr_ind] = curr_sum;
            else
                vertical_sums[curr_ind] = unsmoothed_data[curr_ind];
            vertical_sum_counts[curr_ind] = sum_count;
        }
    }

    std::vector<int8_t> smoothed_data(unsmoothed_data.size()); // sums / counts to get the actual means
    for (int i = 0; i < horizontal_sums.size(); i++) {
        if (unsmoothed_data[i] < 0 || unsmoothed_data[i] == 100) {
            smoothed_data[i] = unsmoothed_data[i];
        }else {
            smoothed_data[i] = vertical_sums[i] / vertical_sum_counts[i];
        }
    }
    return smoothed_data;
}


// update dilated map based on current data_ value
void Map::updateDilated(uint32_t radius) {
    std::vector<int8_t> data_dilated = generateDilated_(radius, data_);


    // std::vector<int8_t> data_smoothed = generateSmoothed_(5, data_dilated);
    // for (int i=0; i < data_smoothed.size(), i++) {
    //     data_smoothed_[]
    // }
    
    
    data_smoothed_ = generateSmoothed_(5, data_dilated);
}

void Map::plotSmoothedMap(ros::Publisher publisher) {
    nav_msgs::OccupancyGrid message;
    message.header.frame_id = "/smoothed_map";
//    message.header.stamp = ros::Time::now();
    message.data = data_smoothed_;
    message.info = map_info_;
    publisher.publish(message);
}


// find the closest frontier to the given x and y coordinates
std::vector<std::pair<float, float>> Map::closestFrontier(float xf, float yf)
{
    

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
        std::vector<std::pair<float, float>> emptyMap;
        return emptyMap;
    }

    Map::AdjacencyRelationship rel = adjacencyGrid_[x][y];

    // set starting tile to visited, push it to the queue
    visited[x][y] = true;
    queue.push(rel);

    // is the current frontier bigger than the minimum defined size?
    bool bigEnough = false; 

    // process every tile in the queue or until a big enough frontier is found
    while (queue.size() != 0 && !bigEnough) {
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

                ROS_INFO("finished BFS, found frontier at x=%u, y=%u", rel.self.x, rel.self.y);

                // perform DFS and propagate along border to check size

                //////////////////////////////////////////////////////
                //            Border Propagation (DFS)              //
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

                // check border size, return those tiles if the border is big enough
                //ROS_INFO("traversed border and found %lu tiles", border.size());

                if (border.size() >= MINBORDERSIZE) {
                    std::vector<std::pair<float, float>> frontierCoords;

                    for (auto frontier: border) {
                        frontierCoords.push_back(mapToPos(frontier.x, frontier.y));
                    }
                    // uncomment this to plot robot coords as well:
                    // frontierCoords.insert(std::pair<float, float>(xf, yf));

                    // set goal coordinate for sebastian
                    frontier_ = frontierCoords[(int)frontierCoords.size()];

                    return frontierCoords;
                }
                // otherwise we find the next frontier

            }
            // if tile at these coords has not been visited, add to the queue and set to visited
            // tile must also be open space in order to be added, so that search does not go through walls
            // this is part of the BFS
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
    // if no frontier big enough was found, return an empty map of floats
    std::vector<std::pair<float, float>> emptyfloatMap;
    return emptyfloatMap;
    

}

// returns a path for robot to follow to the frontier
// takes X and Y robot positions, returns vector of coordinates
// makes use of private members occupancy grid (pre-processed version) and frontier coordinates

std::vector<std::pair<float, float>> Map::getPath(float posX, float posY) 
{
    //std::cout << "entered getPath" << std::endl;
    std::pair<uint32_t, uint32_t> robotStart = posToMap(posX, posY);

    uint32_t endX = frontier_.first, endY = frontier_.second;

    std::map<uint32_t, Map::Tile_Info> tileMap;

    // if (smoothedAdjacencyGrid_[robotStart.first][robotStart.second].self.occ == nullptr) {
    //     std::cout << "null pointer in smooth grid" << std::endl;
    // }

    std::cout << "before deref" << std::endl;
    int8_t occ_val = *smoothedAdjacencyGrid_[robotStart.first][robotStart.second].self.occ;
    std::cout << "after deref" << std::endl;


    // creates initial struct entry for the starting position
    Map::Tile_Info start(robotStart.first, robotStart.second, endX, endY,
                         *smoothedAdjacencyGrid_[robotStart.first][robotStart.second].self.occ);
    start.parent = &start;

    // set the current tile to start
    Map::Tile_Info curTile = start;

    // loop to check every adjascent tile and create a struct entry for it
    // run until it reaches frontier tile
    while (!tileMap.empty()) {

        // set current tile as the one with lowest total cost in tileMap
        float lowestCost = 9999999.0;
        for (auto tilePair: tileMap) {
            if (!tilePair.second.checked && tilePair.second.totalCost < lowestCost) {
                lowestCost = tilePair.second.totalCost;
                Map::Tile_Info curTile = tilePair.second;
            }
        }
        curTile.checked = true; // set current tile to checked

        // exit loop if current tile is frontier
        if (curTile.x == endX && curTile.y == endY) break;

        uint32_t x = curTile.x, y = curTile.y;
 
        // iterate over neighbours of curTile
        for (int i = -1; i<= 1; i++) {
            for (int j = -1; j<= 1; j++) {
                if (i && j) {
                    uint32_t nx = x + i, ny = y + j; // neighbour coords
                    if (nx < 0 || nx > width_ - 1 || ny < 0 || ny > height_ - 1) {
                        ROS_WARN("A* adjacent tile is out of map scope, skipping!");
                        continue;
                    }
                    int8_t occ = *smoothedAdjacencyGrid_[nx][ny].self.occ; // neighbour weight

                    // grab neighbour iterator to check if neighbour exists in map, check traversability
                    std::map<uint32_t, mainSpace::Map::Tile_Info>::iterator curNeighbourIt = tileMap.find(nx + width_*ny);
                    bool inMap = curNeighbourIt != tileMap.end();
                    bool valid = occ != -1 && occ != 100;

                    // if not traversable OR (neighbour exists AND has been checked), skip it
                    // have to check if neighbour exists before getting its checked status
                    if (!valid || (inMap && curNeighbourIt->second.checked)) continue;

                    // create new neighbour, decide later if it is new or should replace current one
                    Map::Tile_Info newNeighbour(nx, ny, endX, endY, &curTile, occ);

                    // if no neigbour at these coords, add neighbour to map
                    if (!inMap) {
                        tileMap.emplace(nx + width_*ny, newNeighbour);
                    }
                    else {
                        // grab current neighbour for path length comparison
                        Map::Tile_Info curNeighbour = curNeighbourIt->second;
                        if (newNeighbour.pathLength < curNeighbour.pathLength) {
                            tileMap.erase(nx + width_*ny);
                            tileMap.emplace(nx + width_*ny, newNeighbour);
                        }
                    }
                    
                }
            }
        }


    }

    std::vector<std::pair<float, float>> backPath;

    while (curTile.parent != &curTile) {
        backPath.push_back(mapToPos(curTile.x, curTile.y));
        curTile = *curTile.parent;
    }
    // std::cout << "left getPath" << std::endl;
    return backPath;

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

    // std::cout << "floatX: " << floatX << std::endl;
    // std::cout << "mapXPos: " << map_info_.origin.position.x << std::endl;
    // std::cout << "floatY: " << floatY << std::endl;
    // std::cout << "mapYPos: " << map_info_.origin.position.y << std::endl;
    // std::cout << "mapRes: " << map_info_.resolution << std::endl;


    float xt = (floatX - map_info_.origin.position.x) / map_info_.resolution;
    // std::cout << "xt: " << xt << std::endl;
    if (xt >= 0) x = (uint32_t)ceil(xt);
    else ROS_WARN("xt=%f, will use 0 instead", xt);
    float yt = (floatY - map_info_.origin.position.y) / map_info_.resolution;
    // std::cout << "yt: " << yt << std::endl;
    if (yt >= 0) y = (uint32_t)ceil(yt);
    else ROS_WARN("yt=%f, will use 0 instead", yt);

    // std::cout << "x: " << x << std::endl;
    // std::cout << "y: " << y << std::endl;

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