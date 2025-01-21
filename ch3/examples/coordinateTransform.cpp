/**
 * @file coordinateTransform.cpp
 * @author chenyinjie
 * @date 2024-10-19
 */

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>


#include <iostream>
#include <iomanip>

int main() {
    Eigen::Quaternion q1(0.55, 0.3, 0.2, 0.2);
    Eigen::Quaternion q2(-0.1, 0.3, -0.7, 0.2);
    Eigen::Vector3d t1(0.7, 1.1, 0.2);
    Eigen::Vector3d t2(-0.1, 0.4, 0.8);

    q1.normalize();
    q2.normalize();

    // Create a 4x4 transformation matrix
    Eigen::Isometry3d T1w(q1);
    Eigen::Isometry3d T2w(q2);

    // set translation
    T1w.pretranslate(t1);
    T2w.pretranslate(t2);

    // Create a vector and transform it
    Eigen::Vector3d p1(0.5, 0, 0.2);

    Eigen::Vector3d p2 = T2w * T1w.inverse() * p1;
    std::cout << p2 << std::endl;
    std::cout << "----------------" << std::endl;
    std::cout << p2.transpose() << std::endl;
      
    return 0;
}