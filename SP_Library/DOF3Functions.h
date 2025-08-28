#pragma once
#include <Arduino.h>
#include "MathUtils.h"

void angles2motor_3DOF(double q1, double q2, double q3, double& m1, double& m2, double& m3);
int mm_to_increments(double mm);
double increments_to_mm(int increments);
void FK_3DOF(double m1, double m2, double m3, double& x, double& y, double& z, double& q1, double& q2, double& q3, double& q4);
Matrix<3, 3> J_3DOF(double m1, double m2, double m3, double q1, double q2, double q3, double q4);
void IK_3DOF(const Matrix<3> Target, Matrix<3>& motor_values, Matrix<4>& joint_angles);
void R2q3(double target_dist, const Matrix<3>& kf, const Matrix<3>& kh, const Matrix<3>& hg, double l5, const Matrix<3>& d1, double d2, double d3x, double d3y, double& q3_out, double& q4_out);
double q3_2_q4(double q3, const Matrix<3>& kf, const Matrix<3>& kh, const Matrix<3>& hg, double l5);
double range_fn(double q3, double q4, const Matrix<3>& d1, double d2, double d3x, double d3y);
double dR_dq3(double R, double q3, double q4, const Matrix<3>& d1, double d2, double d3x, double d3y, const Matrix<3>& kf, const Matrix<3>& kh, const Matrix<3>& hg);
void scaleMotorVelocities(const Matrix<3>& v_in, double vmax, Matrix<3>& v_out);
