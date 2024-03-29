#include "ros/ros.h"
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include <map>
#include <queue>
#include <stack>
#include <chrono>

#define MINBORDERSIZE 3

namespace mainSpace {

class Map {

    struct Tile {
        uint32_t x;
        uint32_t y;
        uint32_t px;
        uint32_t py;
        int8_t* occ;
        Tile(uint32_t xt, uint32_t yt);
    };

    struct AdjacencyRelationship {
        Tile self;
        std::vector<Tile> adjacentTiles;
        AdjacencyRelationship(Tile selft, std::vector<Tile> adjt);
    };

    struct Tile_Info {
        uint32_t x;
        uint32_t y;
        uint32_t px;
        uint32_t py;
        float pathLength;
        float endDist;
        float totalCost;
        bool checked;

        float weightFactor = 0.01;

        // constructor without parent (for start)
        Tile_Info(uint32_t xt, uint32_t yt, uint32_t end_x, uint32_t end_y, int8_t occ);
        // constructor with parent
        Tile_Info(uint32_t xt, uint32_t yt, uint32_t end_x, uint32_t end_y, 
                  uint32_t pxt, uint32_t pyt, float pLength, int8_t occ);
    };

private:
    nav_msgs::MapMetaData map_info_;
    uint32_t width_;
    uint32_t height_;
    std::vector<int8_t> data_;
    std::vector<int8_t> data_smoothed_;

    // 2D vector of mappings between Tiles and their adjacent Tiles
    std::vector<std::vector<AdjacencyRelationship>> adjacencyGrid_;
    std::vector<std::vector<AdjacencyRelationship>> smoothedAdjacencyGrid_;

    // closest frontier map coordinates
    std::pair<uint32_t, uint32_t> frontier_;

    std::vector<int8_t> generateDilated_(uint32_t radius, std::vector<int8_t> unsmoothed_data);
    std::vector<int8_t> generateSmoothed_(uint32_t kernel_size, std::vector<int8_t> unsmoothed_data);
    void createAdjacencyRelationship_(std::vector<int8_t> data,
                                      std::vector<std::vector<AdjacencyRelationship>> *adjacencyGrid);
    std::vector<int8_t> padData_(uint32_t amount, std::vector<int8_t> data);

public:
    Map();
    Map(nav_msgs::MapMetaData map_info);
    //~Map();
    void info();
    void update(std::vector<int8_t> data);
    void updateDilated(uint32_t radius);
    std::vector<std::pair<float, float>> closestFrontier(float xf, float yf, std::vector<std::pair<float, float>>* path_);
    std::vector<std::pair<float, float>> getPath(float posX, float posY);
    nav_msgs::MapMetaData getInfo();
    uint32_t getWidth();
    uint32_t getHeight();
    std::pair<float, float> mapToPos(uint32_t intX, uint32_t intY);
    std::pair<uint32_t, uint32_t> posToMap(float floatX, float floatY);

    std::vector<std::pair<uint32_t, uint32_t>> invis;

    // deprecated
    void print();
    // deprecated
    std::vector<Tile> getAdjacent(Tile s);
    // deprecated
    std::vector<std::vector<bool>> frontierScan();

    void plotSmoothedMap(ros::Publisher publisher);
};

} // namespace end