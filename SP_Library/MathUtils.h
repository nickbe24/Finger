#pragma once
#include <Arduino.h>
#include <BasicLinearAlgebra.h>

using namespace BLA;

void rot_x(double angle, Matrix<3, 3>& R);
void rot_y(double angle, Matrix<3, 3>& R);
void rot_z(double angle, Matrix<3, 3>& R);
double Dot(const Matrix<3>& a, const Matrix<3>& b);
double deg2rad(double degrees);
double rad2deg(double rads);
Matrix<4,4> multiplyByScalar_4x4(const Matrix<4,4>& mat, double scalar);
void printMatrix4x4(const Matrix<4,4>& mat);
Matrix<3,3> pinv(const Matrix<3,3>& A, double eps = 1e-8);
