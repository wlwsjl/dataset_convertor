#include<iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

using namespace std;
using namespace Eigen;

int main(int argc, char** argv)
{
    // T_SPAD_mark
    Eigen::Isometry3d T;
    T.linear()(0, 0)   = -0.315817;
    T.linear()(0, 1)   = -0.0312487;
    T.linear()(0, 2)   = -0.948305;
    T.translation()(0) = -0.148274;
    T.linear()(1, 0)   = 0.32418;
    T.linear()(1, 1)   = -0.94286;
    T.linear()(1, 2)   = -0.0768933;
    T.translation()(1) = -0.131504;
    T.linear()(2, 0)   = -0.891721;
    T.linear()(2, 1)   = -0.331706;
    T.linear()(2, 2)   = 0.307903;
    T.translation()(2) = -0.170975;

    // cout << T.rotation() << endl;
    // cout << T.translation() << endl;

    Eigen::Isometry3d T_inv = T.inverse();

    // T_FLIR_mark
    Eigen::Isometry3d T_F;
    T_F.linear()(0, 0)   = -0.306632;
    T_F.linear()(0, 1)   = -0.021832;
    T_F.linear()(0, 2)   = -0.951578;
    T_F.translation()(0) = -0.0330582;
    T_F.linear()(1, 0)   = 0.335431;
    T_F.linear()(1, 1)   = -0.938079;
    T_F.linear()(1, 2)   = -0.0865654;
    T_F.translation()(1) = -0.125525;
    T_F.linear()(2, 0)   = -0.890765;
    T_F.linear()(2, 1)   = -0.345732;
    T_F.linear()(2, 2)   = 0.294968;
    T_F.translation()(2) = -0.15163;

    // cout << T_F.rotation() << endl;
    // cout << T_F.translation() << endl;

    Eigen::Isometry3d T_FLIR_SPAD = T_F * T_inv;

    cout << T_FLIR_SPAD.translation() << endl;
    cout << T_FLIR_SPAD.rotation() << endl;
    cout << T_FLIR_SPAD.translation().norm() << endl;
    cout << T_FLIR_SPAD.rotation().determinant() << endl;

    return 0;
}