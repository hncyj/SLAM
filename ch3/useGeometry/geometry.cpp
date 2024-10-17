/**
 * @file geometry.cpp
 * @author chenyinjie
 * @date 2024-10-17
 */

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace std;
using namespace Eigen;

int main() {
    Matrix3d rotation_matrix = Matrix3d::Identity();
    // typedef AngleAxis<double> AngleAxisd;
    AngleAxisd rotation_vector(M_PI/4, Vector3d(0, 0, 1));
    cout << fixed << setprecision(3) << rotation_vector.matrix() << endl;

    cout << "=========================================\n";

    rotation_matrix = rotation_vector.toRotationMatrix();
    cout << "rotation matrix: \n" << rotation_matrix << endl;

    cout << "=========================================\n";
    Vector3d v(1, 0, 0);
    // 使用旋转向量
    Vector3d v_rotated = rotation_vector * v;
    cout << v.transpose() << " after rotated: " << v_rotated.transpose() << endl;
    cout << "=========================================\n";
    // 使用旋转矩阵
    v_rotated = rotation_matrix * v;
    cout << v.transpose() << " after rotated: " << v_rotated.transpose() << endl;
    cout << "=========================================\n";
    
    // 使用欧拉角描述旋转
    // inline Eigen::Vector3d 
    // Eigen::MatrixBase<Eigen::Matrix3d>::eulerAngles(Eigen::Index a0, Eigen::Index a1, Eigen::Index a2) const;
    // Each of the three parameters a0,a1,a2 represents the respective rotation axis as an integer in {0,1,2}.
    // 2, 1, 0 的顺序表示ZYX - rpy 旋转
    Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0);
    cout << "rpy: " << euler_angles.transpose() << endl;
    cout << "=========================================\n";

    // 欧式变换矩阵
    Isometry3d T = Isometry3d::Identity();
    T.rotate(rotation_vector); // 根据旋转向量设置变换矩阵中表示旋转部分的旋转矩阵
    T.pretranslate(Vector3d(1, 3, 4)); // 设置变换矩阵的平移向量
    cout << "Transform matrix: \n" << T.matrix() << endl;

    cout << "=========================================\n";
    // 利用变换矩阵进行变换
    Vector3d v_transformed = T * v;
    cout << "v after T's transformed:\n" << v_transformed.transpose() << endl;
    cout << "=========================================\n";

    // 四元数
    Quaterniond q = Quaterniond(rotation_vector);
    cout << "q:\n" << q.coeffs() << endl; // coeffs把虚部写在前面，实部在后面
    cout << "=========================================\n";

    // 也可以通过旋转矩阵设置四元数
    q = Quaterniond(rotation_matrix);
    cout << "q:\n" << q.coeffs() << endl;
    cout << "=========================================\n";

    v_rotated = q * v; // 这里重载了 * 表示 qvq^{-1}
    cout << "q rotation: " << v_rotated.transpose() << endl;

    return 0;
}