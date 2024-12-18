/**
 * @file useSophus.cpp
 * @author chenyinjie
 * @date 2024-10-26
 */

#include <iostream>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "sophus/se3.hpp"

using namespace std;
using namespace Eigen;

int main(int argc, char** argv) {
    // rotation vector to rotation matrix: \theta = 90\degree, n = z
    Matrix3d R = AngleAxisd(M_PI / 2, Vector3d(0, 0, 1)).toRotationMatrix();
    // Quaternion
    Quaterniond q(R);
    Sophus::SO3d SO3d_R(R);
    Sophus::SO3d SO3d_q(q);

    cout << "SO(3) from matrix:\n" << SO3d_R.matrix() << endl;
    cout << "--------------" << endl;
    cout << "SO(3) from quaternion:\n" << SO3d_q.matrix() << endl;
    cout << "--------------" << endl;
    Vector3d v3d_R = SO3d_R.log();
    Vector3d v3d_q = SO3d_q.log();
    cout << "SO3d_R to v3d_R: \n" << v3d_R << endl;
    cout << "--------------" << endl;
    cout << "SO3d_q to v3d_q: \n" << v3d_q << endl;

    cout << "--------------" << endl;
    cout << "v3d_R hat (invert symmetry matrix): \n" << Sophus::SO3d::hat(v3d_R) << endl;
    cout << "--------------" << endl;
    cout << "v3d_q hat (invert symmetry matrix): \n" << Sophus::SO3d::hat(v3d_q) << endl;
    cout << "--------------" << endl;
    cout << "v3d_r hat to vee (vector to invert symmetry matrix): \n" << Sophus::SO3d::vee(Sophus::SO3d::hat(v3d_R)) << endl;
    cout << "--------------" << endl;

    // 扰动变换
    Vector3d update_v3d(1e-4, 0, 0);
    // 注意下述 exp 直接接受一个李代数向量
    // 在李代数指数运算的数学表示上，它接受的是一个反对称矩阵，但是exp()函数内置了向量到反对称的转换
    // 参考下述函数的注释：
    // static inline Sophus::SO3d Sophus::SO3d::exp(const Eigen::Vector3d &omega)
    // Group exponential
    // This functions takes in an element of tangent space (= rotation vector
    // omega) and returns the corresponding element of the group SO(3).
    // To be more specific, this function computes expmat(hat(omega))
    // with expmat(.) being the matrix exponential and hat(.) being the
    // hat()-operator of SO(3).
    Sophus::SO3d SO3d_update = Sophus::SO3d::exp(update_v3d) * SO3d_R;
    cout << "updated new rotation matrix: \n" << SO3d_update.matrix() << endl;
    cout << "--------------" << endl;

    // SE(3)
    // 为欧式群表示的变换矩阵添加平移向量
    Vector3d t(1, 0, 0);
    Sophus::SE3d SE3d_R_t(R, t);
    Sophus::SE3d SE3d_q_t(q, t);

    cout << "SE3 from R,t= \n" << SE3d_R_t.matrix() << endl;
    cout << "--------------" << endl;
    cout << "SE3 from q,t= \n" << SE3d_q_t.matrix() << endl;
    cout << "--------------" << endl;

    using Vector6d = Matrix<double, 6, 1>;
    Vector6d se3d_R_t = SE3d_R_t.log();
    cout << "se3d_R_t = \n" << se3d_R_t << endl;
    cout << "--------------" << endl;
    cout << "se3d_R_t hat: \n" << Sophus::SE3d::hat(se3d_R_t) << endl;
    cout << "--------------" << endl;
    cout << "se3d_R_t hat to vee: \n" << Sophus::SE3d::vee(Sophus::SE3d::hat(se3d_R_t)) << endl;

    Vector6d update_se3d_R_t;
    update_se3d_R_t.setZero();
    update_se3d_R_t(0, 0) = 1e-4;
    Sophus::SE3d SE3d_update = Sophus::SE3d::exp(update_se3d_R_t) * SE3d_R_t;
    cout << "--------------" << endl;
    cout << "updated new transfer matrix: \n" << SE3d_update.matrix() << endl;

    return 0;
}

