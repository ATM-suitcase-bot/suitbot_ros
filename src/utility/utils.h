#pragma once

#include <math.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace Eigen;

// wrap to [-pi, pi)
double warpAngle(double x_){
    double x = fmod(x_ + M_PI, 2 * M_PI);
    if (x < 0)
        x += (2*M_PI);
    return x - M_PI;
}

void T2tq(const Matrix4f &T, Vector3f &t, Quaternionf &q)
{
  Eigen::Quaternionf q_(T.topLeftCorner<3, 3>());
  q = q_;

  t[0] = T(0, 3);
  t[1] = T(1, 3);
  t[2] = T(2, 3);
}

void invT(const Matrix4f &T, Matrix4f &T_inv)
{
  T_inv(3, 3) = 1.;
  T_inv.bottomLeftCorner<1, 3>().fill(0.);
  Matrix3f RT = T.topLeftCorner<3, 3>().transpose();
  T_inv.topLeftCorner<3, 3>() = RT;
  T_inv.topRightCorner<3, 1>() = -RT * T.topRightCorner<3, 1>();
}

Matrix4f invT(const Matrix4f &T)
{
  Matrix4f T_inv;
  invT(T, T_inv);
  return T_inv;
}

void bound_2d_array(vector<vector<int>> &arr, int value)
{
    int rows = arr.size();
    int cols = arr[0].size();
    for (int i = 0; i < rows; i++)
    {
        arr[i][0] = value;
        arr[i][cols-1] = value;
    }
    for (int i = 0; i < cols; i++)
    {
        arr[0][i] = value;
        arr[rows-1][i] = value;
    }
}


void floodFillUtil(vector<vector<int>> &arr, int r, int c, int x, int y, int obsC, int newC)
{
    // Base cases
    if (x < 0 || x >= r || y < 0 || y >= c)
        return;
    if (arr[x][y] == obsC)
        return;
    if (arr[x][y] == newC)
        return;
 
    arr[x][y] = newC;
 
    // Recur for north, east, south and west
    floodFillUtil(arr, r, c, x+1, y, obsC, newC);
    floodFillUtil(arr, r, c, x-1, y, obsC, newC);
    floodFillUtil(arr, r, c, x, y+1, obsC, newC);
    floodFillUtil(arr, r, c, x, y-1, obsC, newC);
}
 
// It mainly finds the previous color on (x, y) and
// calls floodFillUtil()
void floodFill(vector<vector<int>> &arr, int r, int c, int x, int y, int obsC, int newC)
{
    if (arr[x][y] == obsC) return;
    floodFillUtil(arr, r, c, x, y, obsC, newC);
}


// a*x+b*y+c*z+d=0
void three_point_plane(Vector3f &A, Vector3f &B, Vector3f &C, float &a, float &b, float &c, float &d)
{
    a = (B[1]-A[1])*(C[2]-A[2])-(C[1]-A[1])*(B[2]-A[2]);
    b = (B[2]-A[2])*(C[0]-A[0])-(C[2]-A[2])*(B[0]-A[0]);
    c = (B[0]-A[0])*(C[1]-A[1])-(C[0]-A[0])*(B[1]-A[1]);
    d = -(a*A[0]+b*A[1]+c*A[2]);
}
