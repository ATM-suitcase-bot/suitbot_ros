#ifndef OCCUPANCY_MAP_H_
#define OCCUPANCY_MAP_H_

#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <iostream>

using namespace std;

typedef enum grid_type_num {
    OCCUPIED = 0,
    FREE = 200, // the actual free space
    UNKNOWN = 255, // might be free but unsensed
    BLOATED = 100
} grid_type_t;


class OccupancyMap
{
public:
	// 2d map
	vector<vector<int>> occupancy_map; // <0 is occupied, 255 is free space, 100 is bloated
	int num_free_cells = 0; // # free cells directly read from the image - bloated cells
    int num_occupied_cells = 0; // computed during obstacle bloating
    int num_bloated_cells = 0; // computed during obstacle bloating
	int rows = 0;
    int cols = 0;

    float resolution = 0.5; // meter

    OccupancyMap(){}
    OccupancyMap(string map_file);

    int initOccupancyGridMap(string map_file);
    int initOccupancyGridMap(string map_file, float res);
    int initOccupancyGridMap(vector<vector<int>> map_in);
    int initOccupancyGridMap(vector<vector<int>> map_in, float res);

    void bloat_obstacles();

    void coord_to_idx(const float &coord_x, const float &coord_y, int &idx_x, int &idx_y);

    void idx_to_coord(const int &ind_x, const int &ind_y, float &coord_x, float &coord_y);

    bool isInMap(const float &px, const float &py);

    vector<int> flatten();

};

#endif