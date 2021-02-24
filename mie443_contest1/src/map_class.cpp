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

    px = -1;
    py = -1;

    int32_t xDist = ((int32_t)end_x - (int32_t)x);
    int32_t yDist = ((int32_t)end_y - (int32_t)y);

    pathLength = weightFactor * occ;
    endDist = sqrt((float)(xDist*xDist + yDist*yDist));
    totalCost = pathLength + endDist;
    //parent = nullptr;
    checked = false;
}

// constructor with parent
Map::Tile_Info::Tile_Info(uint32_t xt, uint32_t yt, uint32_t end_x, uint32_t end_y, 
                          uint32_t pxt, uint32_t pyt, float pLength, int8_t occ)
    :x(xt),
    y(yt),
    px(pxt),
    py(pyt)
{
    int32_t xDist = ((int32_t)end_x - (int32_t)x);
    int32_t yDist = ((int32_t)end_y - (int32_t)y);

    pathLength = pLength
                + sqrt(abs(x - px) + abs(y - py))
                + weightFactor * occ;
    endDist = sqrt((float)(xDist*xDist + yDist*yDist));
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
        gridVals[data_smoothed_[i]]++;
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
        data_[xBump + width_*(yBump)] = 100; 
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
            if (smoothed_data[i] < 0 || smoothed_data[i] > 100) ROS_WARN("smoothed data out of bounds");
        }
    }
    return smoothed_data;
}


// update dilated map based on current data_ value
void Map::updateDilated(uint32_t radius) {
    std::vector<int8_t> data_dilated = generateDilated_(radius, data_);
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
std::vector<std::pair<float, float>> Map::closestFrontier(float xf, float yf, std::vector<std::pair<float, float>>* path_)
{
    
    path_->clear();

    //ROS_INFO("entered closest frontier algorithm");

    //////////////////////////////////////////////////////
    //                  BFS Algorithm                   //
    //////////////////////////////////////////////////////

    // 2D vector of booleans to keep track of visited tiles
    std::vector<bool> visited(width_*height_, false);

    std::pair<uint32_t, uint32_t> mapCoords = posToMap(xf, yf);
    uint32_t x = mapCoords.first;
    uint32_t y = mapCoords.second;
    
    // queue to store adjacency relationships (tiles and their adjacencies)
    std::queue<Map::AdjacencyRelationship> queue;

    ROS_INFO("robot float coordinates are x=%f, y=%f", xf, yf);

    if (x>smoothedAdjacencyGrid_.size() || y>smoothedAdjacencyGrid_[0].size()) {
        ROS_ERROR("BFS starting point of x=%u, y=%u is out of bounds in grid of dims x=%lu, y=%lu",
                    x, y, smoothedAdjacencyGrid_.size(), smoothedAdjacencyGrid_[0].size());
        std::vector<std::pair<float, float>> emptyMap;
        return emptyMap;
    }

    Map::AdjacencyRelationship rel = adjacencyGrid_[x][y];
    adjacencyGrid_[x][y].self.px = 0; adjacencyGrid_[x][y].self.py = 0;

    // set starting tile to visited, push it to the queue
    visited[x + width_*y] = true;
    queue.push(rel);

    // is the current frontier bigger than the minimum defined size?
    //bool bigEnough = false; 

    // process every tile in the queue or until a big enough frontier is found
    while (!queue.empty()) {
        // set rel to the first queue item and then pop it from the queue

        rel = queue.front();
        queue.pop();

        // get the adjacent tiles to this tile
        std::vector<Map::Tile> adj = rel.adjacentTiles;

        ROS_INFO("queue size: %li, num neighbours: %li", queue.size(), adj.size());

        //bool relOpen = *rel.self.occ != -1 && *rel.self.occ != 100;
        
        // iterate over its adjacent tiles
        for (auto a: adj) {
            // if the tile in rel is an open space and it has an adjacent unexplored tile
            // this means that the rel tile belongs to a frontier cluster
            
            if (*rel.self.occ == 0 && *a.occ == -1) {

                std::cout << "found frontier, tracing back" << std::endl;

                uint32_t endX = rel.self.x; uint32_t endY = rel.self.y;

                //Map::Tile frontierTile = rel.self;

                //std::cout << "segfault check 2" << std::endl;

                //frontier_ = std::pair<uint32_t, uint32_t>(frontierTile.x, frontierTile.y);

                //std::cout << "segfault check 3" << std::endl;

                while (rel.self.px != 0 || rel.self.py != 0) {
                    ROS_INFO("tracing back from (%u, %u) to (%u, %u), current occ=%i",
                             rel.self.x, rel.self.y, rel.self.px, rel.self.py, *rel.self.occ);
                    //std::cout << "segfault check 1" << std::endl;
                    path_->push_back(mapToPos(rel.self.x, rel.self.y));
                    //std::cout << "segfault check 2" << std::endl;
                    if (rel.self.px < 0 || rel.self.px > width_-1 || rel.self.py < 0 || rel.self.py > height_-1) {
                        std::cout << "parent coords out of range" << std::endl;
                        path_->clear();
                        break;
                    }
                    rel = adjacencyGrid_[rel.self.px][rel.self.py];
                }

                std::vector<std::pair<float, float>> retVec;
                retVec.push_back(mapToPos(endX, endY));

                return retVec;
                /*
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
                        if (*a.occ != -1 && *a.occ != 100 && !visitedTiles[a.x + width_*a.y]) {
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

                    //std::cout << "segfault check 1" << std::endl;

                    // set goal coordinate for sebastian
                    //frontier_ = *(frontierCoords.begin() + frontierCoords.size()/2);
                    Map::Tile midTile = border[(int)border.size()/2];

                    //std::cout << "segfault check 2" << std::endl;

                    frontier_ = std::pair<uint32_t, uint32_t>(midTile.x, midTile.y);

                    //std::cout << "segfault check 3" << std::endl;

                    while (rel.self.px != 0 || rel.self.py != 0) {
                        //std::cout << "segfault check 1" << std::endl;
                        path_->push_back(mapToPos(rel.self.x, rel.self.y));
                        //std::cout << "segfault check 2" << std::endl;
                        if (rel.self.px < 0 || rel.self.px > width_-1 || rel.self.py < 0 || rel.self.py > height_-1) break;
                        rel = adjacencyGrid_[rel.self.px][rel.self.py];
                    }

                    //std::cout << "segfault check 4" << std::endl;

                    return frontierCoords;
                }
                // otherwise we find the next frontier
                */
            }
            // if tile at these coords has not been visited, add to the queue and set to visited
            // tile must also be open space in order to be added, so that search does not go through walls
            // this is part of the BFS

            // *a.occ != -1 && *a.occ != 100
            else if (!visited[a.x + width_ * a.y] && *a.occ == 0) {
                // set this tile's position as visited
                visited[a.x + width_ * a.y] = true;
                // push the adjacency relationship to the queue and loop back

                // ROS_INFO("tried pushing adjacencyGrid_ of size %u by %u at indices [%u][%u] to queue",
                //             adjacencyGrid_.size(), adjacencyGrid_[0].size(), a.x, a.y);
                adjacencyGrid_[a.x][a.y].self.px = rel.self.x;
                adjacencyGrid_[a.x][a.y].self.py = rel.self.y;
                queue.push(adjacencyGrid_[a.x][a.y]);
            }
            else {
                std::cout << "invalid neighbour, visited: " << visited[a.x + width_ * a.y]
                          << ", occ: " << (int)*a.occ << std::endl;
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

    std::pair<uint32_t, uint32_t> robotStart = posToMap(posX, posY);
    // std::pair<uint32_t, uint32_t> frontierInt = posToMap(frontier_.first, frontier_.second);

    uint32_t startX = robotStart.first, startY = robotStart.second,
             endX = frontier_.first, endY = frontier_.second;

    // if no frontier available. return empty vector
    if (endX == 0 && endY == 0) {
        ROS_INFO("no frontier available for path planning");
        return std::vector<std::pair<float, float>>();
    }

    //ROS_INFO("running A* search from (%u, %u) to (%u, %u)", startX, startY, endX, endY);

    if (*smoothedAdjacencyGrid_[startX][startY].self.occ == -1 || *smoothedAdjacencyGrid_[startX][startY].self.occ == 100) {
        ROS_WARN("invalid starting tile");
        return std::vector<std::pair<float, float>>();

    }
    if (*smoothedAdjacencyGrid_[endX][endY].self.occ == -1 || *smoothedAdjacencyGrid_[endX][endY].self.occ == 100) {
        ROS_WARN("invalid ending tile");
        return std::vector<std::pair<float, float>>();
    }

    std::map<uint32_t, Map::Tile_Info> tileMap;

    // creates initial struct entry for the starting position
    Map::Tile_Info start(startX, startY, endX, endY,
                         *smoothedAdjacencyGrid_[startX][startY].self.occ);
    
    tileMap.emplace(startX + width_*startY, start);

    // set the current tile to start
    Map::Tile_Info curTile = start;
    //Map::Tile_Info* startAddr = &start;

    // loop to check every adjascent tile and create a struct entry for it
    // run until it reaches frontier tile
    for (int loopCount = 0; loopCount < 10000; loopCount++) {

        // set current tile as the one with lowest total cost in tileMap
        float lowestCost = 9999999.0;
        uint32_t uncheckedCount = 0;
        for (auto tilePair: tileMap) {
            if (!tilePair.second.checked) {
                uncheckedCount++;
                if (tilePair.second.totalCost < lowestCost) {
                    lowestCost = tilePair.second.totalCost;
                    curTile = tilePair.second;
                }
            }
        }

        // exit loop if current tile is frontier
        if (curTile.x == endX && curTile.y == endY) {
            //std::cout << "found destination, going back" << std::endl;
            break;
        }

        if (!uncheckedCount) {
            ROS_WARN("checked all tiles, no available path");
            return std::vector<std::pair<float, float>>();
        }

        uint32_t x = curTile.x, y = curTile.y;

        tileMap.find(x + width_*y)->second.checked = true; // set current tile to checked


        // ROS_INFO("checking tile at (%u, %u) whose parent is at (%u, %u), cost=%f, available=%li", 
        //             x, y, curTile.px, curTile.py, curTile.totalCost, tileMap.size());
        // ros::Duration(0.5).sleep();

        // skip current tile if on the edge
        if (x <= 0 || x >= width_ - 1 || y <= 0 || y >= height_ - 1) continue;
        
 
        // iterate over neighbours of curTile
        for (int i = -1; i <= 1; i++) {
            for (int j = -1; j <= 1; j++) {
                if (i && j) {
                    uint32_t nx = x + i, ny = y + j; // neighbour coords
                    if (nx < 0 || nx > width_ - 1 || ny < 0 || ny > height_ - 1) {
                        ROS_WARN("A* adjacent tile is out of map scope, skipping it");
                        //return std::vector<std::pair<float, float>>();
                        continue;
                    }
                    int8_t occ = *smoothedAdjacencyGrid_[nx][ny].self.occ; // neighbour weight

                    // grab neighbour iterator to check if neighbour exists in map, check traversability
                    std::map<uint32_t, Map::Tile_Info>::iterator curNeighbourIt = tileMap.find(nx + width_*ny);
                    bool inMap = curNeighbourIt != tileMap.end();
                    bool valid = occ != -1 && occ != 100;

                    // if not traversable OR (neighbour exists AND has been checked), skip it
                    // have to check if neighbour exists before getting its checked status
                    if (!valid || (inMap && curNeighbourIt->second.checked)) continue;

                    // create new neighbour, decide later if it is new or should replace current one
                    Map::Tile_Info newNeighbour(nx, ny, endX, endY, x, y, curTile.pathLength, occ);
                    //ROS_INFO("created new neighbour at (%u, %u), with cost=%f", nx, ny, newNeighbour.totalCost);

                    // if no neigbour at these coords, add neighbour to map
                    if (!inMap) {
                        //ROS_INFO("no neighbour here. adding it");
                        tileMap.emplace(nx + width_*ny, newNeighbour);
                    }
                    else {
                        // grab current neighbour for path length comparison
                        Map::Tile_Info curNeighbour = curNeighbourIt->second;
                        //ROS_INFO("current neighbour has cost %f", curNeighbour.totalCost);
                        if (newNeighbour.pathLength < curNeighbour.pathLength) {
                            //ROS_INFO("new neighbour has shorter path, replacing it now");
                            tileMap.erase(nx + width_*ny);
                            tileMap.emplace(nx + width_*ny, newNeighbour);
                        }
                    }
                    
                }
            }
        }


    }

    std::vector<std::pair<float, float>> backPath;

    // ROS_INFO("curTile: (%u, %u), its parent: (%u, %u), and its parent: (%u, %u)",
    //          curTile.x, curTile.y, curTile.parent->x, curTile.parent->y,
    //          curTile.parent->parent->x, curTile.parent->parent->y);

    uint32_t pathCount = 0;

    while (curTile.x != startX && curTile.y != startY && pathCount < 10000) {
        // if (curAddr->parent == endAddr) {
        //     ROS_WARN("we're in a loop");
        //     std::cout << "path count: " << pathCount << std::endl;
        //     break;
        // }
        backPath.push_back(mapToPos(curTile.x, curTile.y));
        curTile = tileMap.find(curTile.px + width_*curTile.py)->second;
        pathCount++;
    }
    if (pathCount == 50000) std::cout << "max path length" << std::endl;
    //std::cout << "left getPath" << std::endl;
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