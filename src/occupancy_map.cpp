#include "occupancy_map.h"
#include <opencv2/opencv.hpp>


OccupancyMap::OccupancyMap(string map_file)
{
    int res = initOccupancyGridMap(map_file);
    if (res < 0)
    {
        std::cerr << "ERROR: Cannot create occupancy map! Exiting." << endl;
    }
}


int OccupancyMap::initOccupancyGridMap(string map_file)
{
    cv::Mat map_img = cv::imread(map_file, cv::IMREAD_GRAYSCALE);
    //cv::imshow("map", map_img);
    //cv::waitKey(0);
    if (!map_img.empty())
    {
        for (int r = 0; r < map_img.rows; r++)
        {
            vector<int> line;
            for (int c = 0; c < map_img.cols; c++)
            {
                // outimg.at<uchar>(239 - y_coord , x_coord) = 0;
                int num;
                if (map_img.at<uchar>(r, c) < 50) num = OCCUPIED; // painted black
                else if (map_img.at<uchar>(r, c) < 200){
                    num = FREE;
                    num_free_cells ++; // painted gray (<200)
                }
                else num = UNKNOWN; // white

                line.push_back(num);
            }
            occupancy_map.push_back(line);
        }
    }
    else
    {
        return -1;
    }
    rows = occupancy_map.size();
    cols = occupancy_map[0].size();
    bloat_obstacles();
    return 0;
}

int OccupancyMap::initOccupancyGridMap(string map_file, float res)
{
    resolution = res;
    return initOccupancyGridMap(map_file);
}

int OccupancyMap::initOccupancyGridMap(vector<vector<int>> map_in)
{
    occupancy_map = map_in;
    rows = occupancy_map.size();
    cols = occupancy_map[0].size();
    // count the cells
    for (int r = 0; r < rows; r++)
    {
        for (int c = 0; c < cols; c++)
        {
            if (occupancy_map[r][c] == OCCUPIED)
                num_occupied_cells++;
            else if (occupancy_map[r][c] == FREE)
                num_free_cells++;
        }
    }
    bloat_obstacles();
    return 0;
}

int OccupancyMap::initOccupancyGridMap(vector<vector<int>> map_in, float res)
{
    resolution = res;
    return initOccupancyGridMap(map_in);
}


void OccupancyMap::bloat_obstacles()
{
    for (int r = 0; r < rows; r++)
    {
        for (int c = 0; c < cols; c++)
        {
            if (occupancy_map[r][c] == OCCUPIED)
            {
                num_occupied_cells++;
                // check all directions
                for (int i = -1; i < 2; i++)
                {
                    int row_new = r + i;
                    for (int j = -1; j < 2; j++)
                    {
                        int col_new = c + j;
                        if ((row_new != r || col_new != c) && row_new >= 0 && row_new < rows && col_new >= 0 && col_new < cols)
                        {
                            if (occupancy_map[row_new][col_new] == FREE)
                            {
                                occupancy_map[row_new][col_new] = BLOATED;
                                num_bloated_cells++;
                            }
                            
                        }
                    }
                }
            }
        }
    }
    num_free_cells -= num_bloated_cells;
    std::cout << "Number of free cells: " << num_free_cells << std::endl;
    std::cout << "Number of occupied cells: " << num_occupied_cells << std::endl;
    std::cout << "Number of bloated cells: " << num_bloated_cells << std::endl;
}


void OccupancyMap::coord_to_idx(const float &coord_x, const float &coord_y, int &idx_x, int &idx_y)
{
    // TODO
    idx_x = (int)(coord_x / resolution);
    idx_y = (int)(coord_y / resolution);
}

// returns the top left corner coordinate, coord_x is horizontal
void OccupancyMap::idx_to_coord(const int &ind_x, const int &ind_y, float &coord_x, float &coord_y)
{
    coord_x = (float)ind_x * resolution;
    coord_y = (float)ind_y * resolution;
}

bool OccupancyMap::isInMap(const float &px, const float &py)
{
    int idx_x, idx_y;
    coord_to_idx(px, py, idx_x, idx_y);
    if (idx_x < 0 || idx_x >= cols || idx_y < 0 || idx_y >= rows)
        return false;
    int val = occupancy_map[idx_y][idx_x];
    if (val == FREE || val == BLOATED)
        return true;
    return false;
}

vector<int> OccupancyMap::flatten()
{
    vector<int> res;
    res.reserve(rows * cols);
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            res.push_back(occupancy_map[i][j]);
    return res;
}