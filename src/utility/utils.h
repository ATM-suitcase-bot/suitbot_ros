#ifndef UTILS_H_
#define UTILS_H_

#include <math.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <stdlib.h>

using namespace std;
using namespace Eigen;

struct hash_pair
{
    template <class T1, class T2>
    size_t operator()(const pair<T1, T2> &p) const
    {
        std::size_t seed = 0;
        seed ^= hash<T1>{}(p.first) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= hash<T2>{}(p.second) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        return seed;
    }
};


// wrap to [-pi, pi)
float warpAngle(float x_);

void T2tq(const Matrix4f &T, Vector3f &t, Quaternionf &q);

void invT(const Matrix4f &T, Matrix4f &T_inv);

Matrix4f invT(const Matrix4f &T);

void bound_2d_array(vector<vector<int>> &arr, int value);


void floodFillUtil(vector<vector<int>> &arr, int r, int c, int x, int y, int obsC, int newC);
 
// It mainly finds the previous color on (x, y) and
// calls floodFillUtil()
void floodFill(vector<vector<int>> &arr, int r, int c, int x, int y, int obsC, int newC);


// a*x+b*y+c*z+d=0
void three_point_plane(Vector3f &A, Vector3f &B, Vector3f &C, float &a, float &b, float &c, float &d);

void array_to_image(vector<vector<int>> &arr, cv::Mat &outimg);

#endif