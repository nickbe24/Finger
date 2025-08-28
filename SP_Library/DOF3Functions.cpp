#include "DOF3Functions.h"
#include "MathUtils.h"


void angles2motor_3DOF(double q1, double q2, double q3, double& m1, double& m2, double& m3) { //takes in degrees (1/2/3) returns inc (1) and mm (2/3)
  m1 = (q1*4096.0*16.0)/360;
  q1 = deg2rad(q1);
  q2 = deg2rad(q2);
  q3 = deg2rad(q3);
  // --- Constants ---
  Matrix<3> qa = {0, 10, 10};
  Matrix<3> qm2 = {0, 13.5, -16.006};
  Matrix<3> qm3 = {0, -1.5, -16.192};
  Matrix<3> qk = {0, 5, 40};
  Matrix<3> ab = {0, -13, 0};
  Matrix<3> ac = {0, -8, 5};
  Matrix<3> kd = {0, 4.5, 5};

  Matrix<3> ak = qk - qa;

  const double l2 = 26.24;
  const double l3 = 26.24;
  const double l4 = 30.92;
  const double l5 = 31.72;

  // --- Joint 1 ---
  Matrix<3,3> R1;
  rot_y(q1, R1);

  // --- Joint 2 (q2 to m2) ---
  Matrix<3,3> R2x, R2;
  rot_x(q2, R2x);
  R2 = R1 * R2x;

  Matrix<3> W2 = qm2 - R2 * qa;
  double temp = sqrt(l2 * l2 - W2(0) * W2(0) - W2(1) * W2(1));
  m2 = -W2(2) + temp;
  if (m2 > 5) m2 = -W2(2) - temp;

  // --- Joint 3 (q3 to beta) ---
  Matrix<3> u1 = ac;
  Matrix<3> u2 = ak;
  Matrix<3> u3 = kd;

  Matrix<3,3> R3;
  rot_x(q3, R3);
  Matrix<3> W3 = u2 + R3 * u3;

  double S = (Dot(W3, W3) + Dot(u1, u1) - l4 * l4) / 2.0;
  double A = u1(1)*W3(2) - u1(2)*W3(1);
  double B = u1(1)*W3(1) + u1(2)*W3(2);
  double denom = sqrt(A*A + B*B);
  double beta = atan2(A, B) + acos(S / denom);
  if (beta > M_PI/2 || beta < -M_PI/4)
    beta = atan2(A, B) - acos(S / denom);

  // --- Joint 3 (beta to m3) ---
  Matrix<3,3> Rb, R_beta;
  rot_x(beta, Rb);
  R_beta = R2 * Rb;

  Matrix<3> W4 = qm3 - R2 * qa - R_beta * ab;
  temp = sqrt(l3 * l3 - W4(0) * W4(0) - W4(1) * W4(1));
  m3 = -W4(2) + temp;
  if (m3 > 5) m3 = -W4(2) - temp;
}


int mm_to_increments(double mm) { // 0.5 mm pitch, 16:1 reduction, 4096 encoder increments
  return static_cast<int>(mm * 2 * 16 * 4096);
}


double increments_to_mm(int increments) {
  return static_cast<double>(increments) / (4096.0 * 2 * 16);
}

void FK_3DOF(double m1, double m2, double m3, double& x, double& y, double& z, double& q1, double& q2, double& q3, double& q4) {  //takes in enc (1), mm (2/3) and returns degrees (1/2/3)
    // Fixed Matrixs
    Matrix<3> qa = {0, 10, 10};
    Matrix<3> qm2 = {0, 13.5, -16.006};
    Matrix<3> qm3 = {0, -1.5, -16.192};
    Matrix<3> qk = {0, 5, 40};
    Matrix<3> ab = {0, -13, 0};
    Matrix<3> ac = {0, -8, 5};
    Matrix<3> kf = {0, -6, 0};
    Matrix<3> kd = {0, 4.5, 5};
    Matrix<3> kh = {0, 0, 30};
    Matrix<3> hg = {0, 4.3, 0};
    Matrix<3> ht = {0, 0, 25};
    Matrix<3> ak = qk - qa;

    // Link lengths
    const double l2 = 26.24;
    const double l3 = 26.24;
    const double l4 = 30.92;
    const double l5 = 31.72;

    // Joint angles
    q1 = (m1*360) / (4096*16); //increments to degrees
    q1 = deg2rad(q1);

    Matrix<3> a = qa;
    Matrix<3> P2 = qm2;
    Matrix<3> U2 = P2 + Matrix<3>({0, 0, m2});
    double S2 = (Dot(U2, U2) + Dot(a, a) - l2*l2) / 2.0;
    double A2 = U2(0)*a(1)*sin(q1) - U2(1)*a(2) + U2(2)*a(1)*cos(q1);
    double B2 = U2(0)*a(2)*sin(q1) + U2(1)*a(1) + U2(2)*a(2)*cos(q1);
    double normAB2 = sqrt(A2*A2 + B2*B2);

    q2 = atan2(A2, B2) + acos(S2 / normAB2);
    if (q2 > deg2rad(30) || q2 < deg2rad(-120)) {
        q2 = atan2(A2, B2) - acos(S2 / normAB2);
    }

    // Bell crank beta
    Matrix<3> b = ab;
    Matrix<3> P3 = qm3;
    Matrix<3> M3 = {0, 0, m3};
    Matrix<3,3> R2 = {
        cos(q1), sin(q1)*sin(q2), sin(q1)*cos(q2),
        0,       cos(q2),        -sin(q2),
       -sin(q1), cos(q1)*sin(q2), cos(q1)*cos(q2)
    };
    Matrix<3> U_beta = P3 + M3 - R2 * a;
    double S_beta = (Dot(U_beta,U_beta) + Dot(b,b) - l3*l3)/2.0;
    double A_beta = U_beta(0)*b(1)*sin(q1) - U_beta(1)*b(2) + U_beta(2)*b(1)*cos(q1);
    double B_beta = U_beta(0)*b(2)*sin(q1) + U_beta(1)*b(1) + U_beta(2)*b(2)*cos(q1);
    double normAB_beta = sqrt(A_beta*A_beta + B_beta*B_beta);

    double beta = atan2(A_beta, B_beta) + acos(S_beta / normAB_beta);
    if (beta > deg2rad(80) || beta < deg2rad(-55)) {
        beta = atan2(A_beta, B_beta) - acos(S_beta / normAB_beta);
    }

    // q3
    Matrix<3,3> R_beta_3;
    rot_x(beta - q2, R_beta_3);
    Matrix<3> V3 = ak - R_beta_3 * ac;
    double S3 = (l4*l4 - Dot(V3,V3) - Dot(kd,kd)) / 2.0;

    double A3 = V3(2)*kd(1) - V3(1)*kd(2);
    double B3 = V3(1)*kd(1) + V3(2)*kd(2);
    double normAB3 = sqrt(A3*A3 + B3*B3);
    q3 = atan2(A3, B3) + acos(S3 / normAB3);
    if (q3 > deg2rad(30) || q3 < deg2rad(-120)) {
        q3 = atan2(A3, B3) - acos(S3 / normAB3);
    }

    // q4
    Matrix<3,3> R3;
    rot_x(q3, R3);
    Matrix<3> U4 = R3 * kh - kf;
    double S4 = (l5*l5 - Dot(U4,U4) - Dot(hg,hg)) / 2.0;
    double A4 = U4(2)*hg(1) - U4(1)*hg(2);
    double B4 = U4(1)*hg(1) + U4(2)*hg(2);
    double normAB4 = sqrt(A4*A4 + B4*B4);
    q4 = atan2(A4, B4) + acos(S4 / normAB4) - q3;
    if (q4 > deg2rad(30) || q4 < deg2rad(-120)) {
        q4 = atan2(A4, B4) - acos(S4 / normAB4) - q3;
    }

    // Forward Kinematics Position
    Matrix<4,4> T1 = {cos(q1), 0, sin(q1), 0,
                      0,      1, 0,       0,
                     -sin(q1),0, cos(q1), 0,
                      0,      0, 0,       1};
    Matrix<4,4> T2 = {1, 0,       0,        0,
                      0, cos(q2), -sin(q2), 0,
                      0, sin(q2), cos(q2),  0,
                      0, 0,       0,        1};
    Matrix<4,4> T3 = {1, 0,       0,        0,
                      0, cos(q3), -sin(q3), qk(1),
                      0, sin(q3), cos(q3),  qk(2),
                      0, 0,       0,        1};
    Matrix<4,4> T4 = {1, 0,       0,        0,
                      0, cos(q4), -sin(q4), kh(1),
                      0, sin(q4), cos(q4),  kh(2),
                      0, 0,       0,        1};
    Matrix<4,4> T5 = {1, 0, 0, 0,
                      0, 1, 0, ht(1),
                      0, 0, 1, ht(2),
                      0, 0, 0, 1};

    Matrix<4,4> T = T1 * T2 * T3 * T4 * T5;
    x = T(0,3);
    y = T(1,3);
    z = T(2,3);
    q1 = rad2deg(q1);
    q2 = rad2deg(q2);
    q3 = rad2deg(q3);
    q4 = rad2deg(q4);
}

Matrix<3, 3> J_3DOF(double m1, double m2, double m3, double q1, double q2, double q3, double q4) { // takes in deg and returns Jacobian
    // === Constants ===
    q1 = deg2rad(q1);
    q2 = deg2rad(q2);
    q3 = deg2rad(q3);
    q4 = deg2rad(q4);
    Matrix<3> qa = {0, 10, 10};
    Matrix<3> qm2 = {0, 13.5, -16.006};
    Matrix<3> qm3 = {0, -1.5, -16.192};
    Matrix<3> qk  = {0, 5, 40};
    Matrix<3> ab  = {0, -13, 0};
    Matrix<3> ac  = {0, -8, 5};
    Matrix<3> kf  = {0, -6, 0};
    Matrix<3> kd  = {0, 4.5, 5};
    Matrix<3> kh  = {0, 0, 30};
    Matrix<3> hg  = {0, 4.3, 0};
    Matrix<3> ht  = {0, 0, 25};

    Matrix<3> d1 = qk;
    Matrix<3> d2 = kh;
    Matrix<3> d3 = ht;

    double l2 = 26.24;
    double l3 = 26.24;
    double l4 = 30.92;
    double l5 = 31.72;

    Matrix<3> a = qa;
    Matrix<3> b = ab;
    Matrix<3> u1 = ac;
    Matrix<3> u2 = qk - qa;
    Matrix<3> u3 = kd;
    Matrix<3> v1 = kf;
    Matrix<3> v2 = kh;
    Matrix<3> v3 = hg;
    Matrix<3> P2 = qm2;
    Matrix<3> P3 = qm3;

    // Find beta since its not returned in FK
    Matrix<3> M3 = {0, 0, m3};
    Matrix<3,3> R2 = {
        cos(q1), sin(q1)*sin(q2), sin(q1)*cos(q2),
        0,       cos(q2),        -sin(q2),
       -sin(q1), cos(q1)*sin(q2), cos(q1)*cos(q2)
    };
    Matrix<3> U_beta = P3 + M3 - R2 * a;
    double S_beta = (Dot(U_beta,U_beta) + Dot(b,b) - l3*l3)/2.0;
    double A_beta = U_beta(0)*b(1)*sin(q1) - U_beta(1)*b(2) + U_beta(2)*b(1)*cos(q1);
    double B_beta = U_beta(0)*b(2)*sin(q1) + U_beta(1)*b(1) + U_beta(2)*b(2)*cos(q1);
    double normAB_beta = sqrt(A_beta*A_beta + B_beta*B_beta);

    double beta = atan2(A_beta, B_beta) + acos(S_beta / normAB_beta) - q2;
    if (beta > deg2rad(80) || beta < deg2rad(-55)) {
        beta = atan2(A_beta, B_beta) - acos(S_beta / normAB_beta) - q2;
    }

    // === Velocity Relations ===
    double dq1_dm1 = 1.0;

    // q2

    double E2 = P2(0) - sin(q1) * sin(q2) * a(1) - sin(q1) * cos(q2) * a(2);
    double G2 = P2(1) - cos(q2) * a(1) + sin(q2) * a(2);
    double H2 = P2(2) + m2 - cos(q1) * sin(q2) * a(1) - cos(q1) * cos(q2) * a(2);
    double I2 = E2 * (sin(q1) * sin(q2) * a(2) - sin(q1) * cos(q2) * a(1))
              + G2 * (sin(q2) * a(1) + cos(q2) * a(2))
              + H2 * (cos(q1) * sin(q2) * a(2) - cos(q1) * cos(q2) * a(1));
    double K2 = -E2 * (cos(q1) * sin(q2) * a(1) + cos(q1) * cos(q2) * a(2))
              + H2 * (sin(q1) * sin(q2) * a(1) + sin(q1) * cos(q2) * a(2));

    double dq2_dm1 = - (K2 / I2) * dq1_dm1;
    double dq2_dm2 = - H2 / I2;
    double dq2_dm3 = 0;
    //Serial.println(dq2_dm1,4);
    //Serial.println(dq2_dm2,4);
    //Serial.println(dq2_dm3,4);

    // beta

    double E3 = P3(0) - sin(q1)*sin(q2)*a(1) - sin(q1)*cos(q2)*a(2)
              - sin(q1)*sin(q2+beta)*b(1) - sin(q1)*cos(q2+beta)*b(2);

    double alpha1 = -cos(q1)*sin(q2)*a(1) - cos(q1)*cos(q2)*a(2)
                  - cos(q1)*sin(q2+beta)*b(1) - cos(q1)*cos(q2+beta)*b(2);

    double gamma1 = -sin(q1)*cos(q2)*a(1) + sin(q1)*sin(q2)*a(2)
                   -sin(q1)*cos(q2+beta)*b(1) + sin(q1)*sin(q2+beta)*b(2);

    double epsilon1 = sin(q1)*b(2)*sin(q2+beta) - sin(q1)*b(1)*cos(q2+beta);

    double G3 = P3(1) - cos(q2)*a(1) + sin(q2)*a(2)
              - cos(q2+beta)*b(1) + sin(q2+beta)*b(2);

    double gamma2 = sin(q2)*a(1) + cos(q2)*a(2)
                  + sin(q2+beta)*b(1) + cos(q2+beta)*b(2);

    double epsilon2 = sin(q2+beta)*b(1) + cos(q2+beta)*b(2);

    double H3 = P3(2) + m3 - cos(q1)*sin(q2)*a(1) - cos(q1)*cos(q2)*a(2)
              - cos(q1)*sin(q2+beta)*b(1) - cos(q1)*cos(q2+beta)*b(2);

    double alpha3 = sin(q1)*sin(q2)*a(1) + sin(q1)*cos(q2)*a(2)
                  + sin(q1)*sin(q2+beta)*b(1) + sin(q1)*cos(q2+beta)*b(2);

    double gamma3 = cos(q1)*sin(q2)*a(2) - cos(q1)*cos(q2)*a(1)
                  + cos(q1)*sin(q2+beta)*b(2) - cos(q1)*cos(q2+beta)*b(1);

    double epsilon3 = cos(q1)*sin(q2+beta)*b(2) - cos(q1)*cos(q2+beta)*b(1);

    double I3 = E3*alpha1 + H3*alpha3;
    double K3 = E3*gamma1 + G3*gamma2 + H3*gamma3;
    double L3 = E3*epsilon1 + G3*epsilon2 + H3*epsilon3;

    double dbeta_dm1 = -(I3/L3)*dq1_dm1 - (K3/L3)*dq2_dm1;
    double dbeta_dm2 = - (K3/L3)*dq2_dm2;
    double dbeta_dm3 = - H3/L3;
    //Serial.println(dbeta_dm1,4);
    //Serial.println(dbeta_dm2,4);
    //Serial.println(dbeta_dm3,4);

    // q3

    double E4 = u2(1) + cos(q3)*u3(1) - sin(q3)*u3(2) - cos(beta)*u1(1) + sin(beta)*u1(2);
    double G4 = u2(2) + sin(q3)*u3(1) + cos(q3)*u3(2) - sin(beta)*u1(1) - cos(beta)*u1(2);
    double I4 = -sin(q3)*u3(1) - cos(q3)*u3(2);
    double K4 = sin(beta)*u1(1) + cos(beta)*u1(2);
    double J4 = cos(q3)*u3(1) - sin(q3)*u3(2);
    double L4 = -cos(beta)*u1(1) + sin(beta)*u1(2);
    double omega3 = - (E4*K4+G4*L4)/(E4*I4+G4*J4);

    double dq3_dm1 = omega3 * dbeta_dm1;
    double dq3_dm2 = omega3 * dbeta_dm2;
    double dq3_dm3 = omega3 * dbeta_dm3;
    //Serial.println(dq3_dm1,4);
    //Serial.println(dq3_dm2,4);
    //Serial.println(dq3_dm3,4);

    // q4

    double E5 = cos(q3)*v2(1) - sin(q3)*v2(2) + cos(q3+q4)*v3(1) - sin(q3+q4)*v3(2) - v1(1);
    double G5 = sin(q3)*v2(1) + cos(q3)*v2(2) + sin(q3+q4)*v3(1) + cos(q3+q4)*v3(2) - v1(2);

    double I5 = -sin(q3)*v2(1) - cos(q3)*v2(2);
    double K5 = -sin(q3+q4)*v3(1) - cos(q3+q4)*v3(2);
    double J5 = cos(q3)*v2(1) - sin(q3)*v2(2);
    double L5 = cos(q3+q4)*v3(1) - sin(q3+q4)*v3(2);
    double omega4 = - (E5*I5 + G5*J5 + E5*K5 + G5*L5)/(E5*K5 + G5*L5);

    double dq4_dm1 = omega4 * dq3_dm1;
    double dq4_dm2 = omega4 * dq3_dm2;
    double dq4_dm3 = omega4 * dq3_dm3;
    //Serial.println(dq4_dm1,4);
    //Serial.println(dq4_dm2,4);
    //Serial.println(dq4_dm3,4);

    // === T1–T5 and dT1_dq1–dT4_dq4 ===
    Matrix<4,4> T1 = { cos(q1), 0, sin(q1), 0,
                      0, 1, 0, 0,
                      -sin(q1), 0, cos(q1), 0,
                      0, 0, 0, 1 };
    Matrix<4,4> T2 = { 1, 0, 0, 0,
                      0, cos(q2), -sin(q2), 0,
                      0, sin(q2), cos(q2), 0,
                      0, 0, 0, 1 };
    Matrix<4,4> T3 = { 1, 0, 0, 0,
                      0, cos(q3), -sin(q3), d1(1),
                      0, sin(q3), cos(q3), d1(2),
                      0, 0, 0, 1 };
    Matrix<4,4> T4 = { 1, 0, 0, 0,
                      0, cos(q4), -sin(q4), d2(1),
                      0, sin(q4), cos(q4), d2(2),
                      0, 0, 0, 1 };
    Matrix<4,4> T5 = { 1, 0, 0, 0,
                      0, 1, 0, d3(1),
                      0, 0, 1, d3(2),
                      0, 0, 0, 1 };

    Matrix<4,4> dT1_dq1 = { -sin(q1), 0, cos(q1), 0,
                             0, 0, 0, 0,
                            -cos(q1), 0, -sin(q1), 0,
                             0, 0, 0, 0 };
    Matrix<4,4> dT2_dq2 = { 0, 0, 0, 0,
                             0, -sin(q2), -cos(q2), 0,
                             0, cos(q2), -sin(q2), 0,
                             0, 0, 0, 0 };
    Matrix<4,4> dT3_dq3 = { 0, 0, 0, 0,
                             0, -sin(q3), -cos(q3), 0,
                             0, cos(q3), -sin(q3), 0,
                             0, 0, 0, 0 };
    Matrix<4,4> dT4_dq4 = { 0, 0, 0, 0,
                             0, -sin(q4), -cos(q4), 0,
                             0, cos(q4), -sin(q4), 0,
                             0, 0, 0, 0 };

    // === Derivatives of T w.r.t. m1, m2, m3 ===
    Matrix<4,4> dT_dm1_a = multiplyByScalar_4x4(dT1_dq1, dq1_dm1);
    dT_dm1_a = dT_dm1_a * T2;
    dT_dm1_a = dT_dm1_a * T3;
    dT_dm1_a = dT_dm1_a * T4;
    dT_dm1_a = dT_dm1_a * T5;

    Matrix<4,4> dT_dm1_b = multiplyByScalar_4x4(dT2_dq2, dq2_dm1);
    dT_dm1_b = T1*dT_dm1_b;
    dT_dm1_b = dT_dm1_b*T3;
    dT_dm1_b = dT_dm1_b*T4;
    dT_dm1_b = dT_dm1_b*T5;

    Matrix<4,4> dT_dm1_c = multiplyByScalar_4x4(dT3_dq3, dq3_dm1);
    Matrix<4,4> T12 = T1*T2;
    dT_dm1_c = T12*dT_dm1_c;
    dT_dm1_c = dT_dm1_c*T4;
    dT_dm1_c = dT_dm1_c*T5;
    
    Matrix<4,4> dT_dm1_d = multiplyByScalar_4x4(dT4_dq4, dq4_dm1);
    Matrix<4,4> T123 = T12*T3;
    dT_dm1_d = T123*dT_dm1_d;
    dT_dm1_d = dT_dm1_d*T5;

    Matrix<4,4> dT_dm1 = dT_dm1_a + dT_dm1_b + dT_dm1_c + dT_dm1_d;

    Matrix<4,4> dT_dm2_a = multiplyByScalar_4x4(dT2_dq2, dq2_dm2);
    dT_dm2_a = T1*dT_dm2_a;
    dT_dm2_a = dT_dm2_a*T3;
    dT_dm2_a = dT_dm2_a*T4;
    dT_dm2_a = dT_dm2_a*T5;

    Matrix<4,4> dT_dm2_b = multiplyByScalar_4x4(dT3_dq3, dq3_dm2);
    dT_dm2_b = T12*dT_dm2_b;
    dT_dm2_b = dT_dm2_b*T4;
    dT_dm2_b = dT_dm2_b*T5;

    Matrix<4,4> dT_dm2_c = multiplyByScalar_4x4(dT4_dq4, dq4_dm2);
    dT_dm2_c = T123*dT_dm2_c;
    dT_dm2_c = dT_dm2_c*T5;

    Matrix<4,4> dT_dm2 = dT_dm2_a +dT_dm2_b + dT_dm2_c;

    Matrix<4,4> dT_dm3_a = multiplyByScalar_4x4(dT3_dq3, dq3_dm3);
    dT_dm3_a = T12*dT_dm3_a;
    dT_dm3_a = dT_dm3_a*T4;
    dT_dm3_a = dT_dm3_a*T5;

    Matrix<4,4> dT_dm3_b = multiplyByScalar_4x4(dT4_dq4, dq4_dm3);
    dT_dm3_b = T123*dT_dm3_b;
    dT_dm3_b = dT_dm3_b*T5;

    Matrix<4,4> dT_dm3 = dT_dm3_a + dT_dm3_b;

    Matrix<3,3> J;
    for (int i = 0; i < 3; ++i) {
        J(i, 0) = dT_dm1(i, 3);
        J(i, 1) = dT_dm2(i, 3);
        J(i, 2) = dT_dm3(i, 3);
    }



    return J;
}

void IK_3DOF(const Matrix<3> Target, Matrix<3>& motor_values, Matrix<4>& joint_angles) {
  // --- Geometry Constants ---
  Matrix<3> qa = {0, 10, 10};
  Matrix<3> qk = {0, 5, 40};
  Matrix<3> ht = {0, 0, 25};
  Matrix<3> kh = {0, 0, 30};
  Matrix<3> kf = {0, -6, 0};
  Matrix<3> hg = {0, 4.3, 0};
  Matrix<3> d1 = qk;

  double l5 = 31.72;
  double d2 = sqrt(Dot(kh, kh));
  double d3x = 0;
  double d3y = sqrt(Dot(ht, ht));
  double target_dist = sqrt(Dot(Target, Target));

  // --- Solve for q3 and q4 using Newton-Raphson ---
  double q3, q4;
  R2q3(target_dist, kf, kh, hg, l5, d1, d2, d3x, d3y, q3, q4);

  // --- Solve q1 ---
  double q1 = atan2(Target(0), Target(2));
  if (q1 > PI / 2) q1 -= PI;
  else if (q1 < -PI / 2) q1 += PI;

  // --- Solve q2 ---
  double Y = d1(1) - d2 * sin(q3) - d3x * sin(q3 + q4 - PI/2) - d3y * sin(q3 + q4);
  double Z = d1(2) + d2 * cos(q3) + d3x * cos(q3 + q4 - PI/2) + d3y * cos(q3 + q4);
  Matrix<3,3> R1;
  rot_y(-q1, R1);
  Matrix<3> Tp = R1 * Target;
  double theta1 = atan2(Z, Y);
  double theta2 = atan2(Tp(2), Tp(1));
  double q2 = theta2 - theta1;

  // --- Store joint angles ---
  q1 = rad2deg(q1);
  q2 = rad2deg(q2);
  q3 = rad2deg(q3);
  q4 = rad2deg(q4);
  joint_angles = { q1, q2, q3, q4 };

  // --- Call your motor conversion function ---
  double m1, m2, m3;
  angles2motor_3DOF(q1, q2, q3, m1, m2, m3);
  motor_values = { m1, m2, m3 };
}

void R2q3(double target_dist, const Matrix<3>& kf, const Matrix<3>& kh, const Matrix<3>& hg,
          double l5, const Matrix<3>& d1, double d2, double d3x, double d3y,
          double& q3_out, double& q4_out) {
  double q3 = -PI / 3;
  double q4 = q3_2_q4(q3, kf, kh, hg, l5);
  double R = range_fn(q3, q4, d1, d2, d3x, d3y);
  double error = target_dist - R;

  const int max_iters = 100;
  int iters = 0;

  while (fabs(error) > 0.01 && iters < max_iters) {
    iters++;
    double f = R - target_dist;
    double df = dR_dq3(R, q3, q4, d1, d2, d3x, d3y, kf, kh, hg);
    if (fabs(df) < 1e-6) break;

    q3 -= f / df;
    q4 = q3_2_q4(q3, kf, kh, hg, l5);
    R = range_fn(q3, q4, d1, d2, d3x, d3y);
    error = target_dist - R;
  }

  q3_out = q3;
  q4_out = q4;
}

double q3_2_q4(double q3, const Matrix<3>& kf, const Matrix<3>& kh, const Matrix<3>& hg, double l5) {
  Matrix<3,3> Rq3;
  rot_x(q3, Rq3);
  Matrix<3> V = Rq3 * kh - kf;

  double V_norm2 = Dot(V, V);
  double hg_norm2 = Dot(hg, hg);
  double S5 = (l5 * l5 - V_norm2 - hg_norm2) / 2.0;

  double p = V(2) * hg(1) - V(1) * hg(2);
  double q = V(1) * hg(1) + V(2) * hg(2);
  double r = -S5;

  double q4 = atan2(p, q) + acos(-r / sqrt(p * p + q * q)) - q3;
  if (q4 > deg2rad(30) || q4 < deg2rad(-120))
    q4 = atan2(p, q) - acos(-r / sqrt(p * p + q * q)) - q3;

  return q4;
}

double range_fn(double q3, double q4, const Matrix<3>& d1, double d2, double d3x, double d3y) {
  double R2 = Dot(d1, d1) + d2 * d2 + d3x * d3x + d3y * d3y
            + 2 * d2 * d3x * sin(q4) + 2 * d2 * d3y * cos(q4)
            - 2 * d2 * d1(1) * sin(q3) + 2 * d2 * d1(2) * cos(q3)
            + (2 * d3x * d1(2) - 2 * d3y * d1(1)) * sin(q3 + q4)
            + (2 * d3x * d1(1) + 2 * d3y * d1(2)) * cos(q3 + q4);

  return sqrt(R2);
}

double dR_dq3(double R, double q3, double q4, const Matrix<3>& d1, double d2, double d3x, double d3y,
              const Matrix<3>& kf, const Matrix<3>& kh, const Matrix<3>& hg) {
  double v1 = sqrt(Dot(kf, kf));
  double v2 = sqrt(Dot(kh, kh));
  double v3 = sqrt(Dot(hg, hg));

  double rhs = -2 * v1 * v2 * cos(q3) - 2 * v1 * v3 * sin(q3 + q4);
  double lhs = -2 * v2 * v3 * cos(q4) + 2 * v1 * v3 * sin(q3 + q4);
  double dq4_dq3 = rhs / lhs;

  double term1 = 2 * d2 * d3x * cos(q4) * dq4_dq3 - 2 * d2 * d3y * sin(q4) * dq4_dq3;
  double term2 = -2 * d2 * d1(1) * cos(q3) - 2 * d2 * d1(2) * sin(q3);
  double term3 = (2 * d3x * d1(2) - 2 * d3y * d1(1)) * cos(q3 + q4) * (1 + dq4_dq3);
  double term4 = -(2 * d3x * d1(1) + 2 * d3y * d1(2)) * sin(q3 + q4) * (1 + dq4_dq3);

  return (term1 + term2 + term3 + term4) / (2.0 * R);
}

void scaleMotorVelocities(const Matrix<3>& v_in, double vmax, Matrix<3>& v_out) {
    double max_actual = max(fabs(v_in(0)), max(fabs(v_in(1)), fabs(v_in(2))));

    if (max_actual > vmax) {
        double scale = vmax / max_actual;
        v_out = static_cast<float>(scale) * v_in;
    } else {
        v_out = v_in;
    }
}

