#include "MathUtils.h"
#include <BasicLinearAlgebra.h>
#include <math.h>
using namespace BLA;

void rot_x(double angle, Matrix<3, 3>& R) {
  R = { 1, 0,           0,
        0, cos(angle), -sin(angle),
        0, sin(angle),  cos(angle) };
}

void rot_y(double angle, Matrix<3, 3>& R) {
  R = { cos(angle), 0, sin(angle),
        0,          1, 0,
       -sin(angle), 0, cos(angle) };
}

void rot_z(double angle, Matrix<3, 3>& R) {
  R = { cos(angle), -sin(angle), 0,
        sin(angle),  cos(angle), 0,
        0,           0,          1 };
}

double Dot(const Matrix<3>& a, const Matrix<3>& b) {
  return a(0)*b(0) + a(1)*b(1) + a(2)*b(2);
}

double deg2rad(double degrees) {
  return degrees * M_PI / 180.0;
}

double rad2deg(double rads){
  return rads * 180.0 / M_PI;
}

Matrix<4,4> multiplyByScalar_4x4(const Matrix<4,4>& mat, double scalar) {
    Matrix<4,4> result;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            result(i, j) = mat(i, j) * scalar;
    return result;
}

void printMatrix4x4(const Matrix<4,4>& mat) {
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            Serial.print(mat(i, j), 4);  // 4 digits of precision
            Serial.print("\t");
        }
        Serial.println();
    }
}

Matrix<3,3> pinv(const Matrix<3,3>& A, double eps) {
    double detA = Determinant(A);
    if (fabs(detA) > eps) {
        Matrix<3,3> A_copy = A;
        Invert(A_copy);
        return A_copy;
    }

    Matrix<3,3> At = ~A;
    Matrix<3,3> AAt = A * At;

    double detAAt = Determinant(AAt);
    if (fabs(detAAt) > eps) {
        Matrix<3,3> AAt_inv = AAt;
        Invert(AAt_inv);
        return At * AAt_inv;
    }

    return Matrix<3,3>();  // fallback: zero matrix
}

