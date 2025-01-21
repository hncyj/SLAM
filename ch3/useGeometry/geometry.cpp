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

    Vector3d v_rotated = rotation_vector * v;
    cout << v.transpose() << " after rotated: " << v_rotated.transpose() << endl;
    cout << "=========================================\n";

    v_rotated = rotation_matrix * v;
    cout << v.transpose() << " after rotated: " << v_rotated.transpose() << endl;
    cout << "=========================================\n";
    
    // inline Eigen::Vector3d 
    // Eigen::MatrixBase<Eigen::Matrix3d>::eulerAngles(Eigen::Index a0, Eigen::Index a1, Eigen::Index a2) const;
    // Each of the three parameters a0,a1,a2 represents the respective rotation axis as an integer in {0,1,2}.
    // 2, 1, 0 means: Z-Y-X (rpy)
    Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0);
    cout << "euler rpy angles: " << euler_angles.transpose() << endl;
    cout << "=========================================\n";

    // Notice that the order of rotation and translation is important
    // rotation and translation are not commutative
    // it will generate different euclidean transformation matrix
    Isometry3d T(rotation_vector);
    T.pretranslate(Vector3d(1, 3, 4));
    // Isometry3d T(Vector3d(1, 3, 4));
    // T.rotate(rotation_vector);

    cout << "Transform matrix: \n" << T.matrix() << endl;

    cout << "=========================================\n";

    Vector3d v_transformed = T * v;
    cout << "v after T's transformed:\n" << v_transformed.transpose() << endl;
    cout << "=========================================\n";


    Quaterniond q = Quaterniond(rotation_vector);
    cout << "q:\n" << q.coeffs() << endl; // coeffs把虚部写在前面，实部在后面
    cout << "=========================================\n";


    q = Quaterniond(rotation_matrix);
    cout << "q:\n" << q.coeffs() << endl;
    cout << "=========================================\n";

    v_rotated = q * v; // reload * operator, q * v = qvq^{-1}
    cout << "v after q rotation: " << v_rotated.transpose() << endl;

    return 0;
}