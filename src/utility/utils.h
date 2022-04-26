#ifndef UTILS_H_
#define UTILS_H_

#include <math.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

// wrap to [-pi, pi)
double warpAngle(double x_);

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