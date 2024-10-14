/**
 * @file eigenMatrix.cpp
 * @author chenyinjie
 * @version 0.1
 * @date 2024-10-14
 */

#include <Eigen/Core>
#include <Eigen/Dense>

#include <iostream>

using namespace std;
using namespace Eigen;

const int MATRIX_SIZE = 50;

int main(int argc, char** argv) {
  Matrix<float, 2, 3> matrix_23;
  Vector3d v_3d;
  Matrix<float, 3, 1> vd_3d;
  Matrix3d matrix_33 = Eigen::Matrix3d::Zero();
  Matrix<double, Dynamic, Dynamic> matrix_dynamic;
  MatrixXd matrix_xd;

  matrix_23 << 1, 2, 3, 4, 5, 6;
  cout << "matrix_23: \n" << matrix_23 << endl;

  cout << "========================================" << endl;

  cout << "print matrix 2x3: " << endl;
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 3; j++)
      cout << matrix_23(i, j) << "\t";
    cout << endl;
  }

  v_3d << 3, 2, 1;
  vd_3d << 4, 5, 6;

  cout << "========================================" << endl;

  Matrix<double, 2, 1> result1 = matrix_23.cast<double>() * v_3d;
  cout << "matrix_23 * v_3d: \n" << result1 << endl;
  cout << "matrix_23 * v_3d Transpose: " << result1.transpose() << endl;

  cout << "========================================" << endl;

  Matrix<float, 2, 1> result2 = matrix_23 * vd_3d;
  cout << "matrix_23 * vd_3d: \n" << result2 << endl;
  cout << "matrix_23 * vd_3d Transpose: " << result2.transpose() << endl;

  // error: static assertion failed: YOU_MIXED_MATRICES_OF_DIFFERENT_SIZES
  // Eigen::Matrix<double, 2, 3> result_wrong_dimension =
  // matrix_23.cast<double>() * v_3d; cout << "wrong dimension: \n" <<
  // result_wrong_dimension << endl;

  cout << "========================================" << endl;

  matrix_33 = Matrix3d::Random();                            // 随机数矩阵
  cout << "random matrix: \n" << matrix_33 << endl;          // 原矩阵
  cout << "transpose: \n" << matrix_33.transpose() << endl;  // 转置
  cout << "sum: " << matrix_33.sum() << endl;                // 各元素和
  cout << "trace: " << matrix_33.trace() << endl;            // 迹
  cout << "times 10: \n" << 10 * matrix_33 << endl;          // 数乘
  cout << "inverse: \n" << matrix_33.inverse() << endl;      // 逆
  cout << "det: " << matrix_33.determinant() << endl;        // 行列式

  cout << "========================================" << endl;

  // Eigen vals and Eigen vectors
  SelfAdjointEigenSolver<Matrix3d> eigen_solver(matrix_33.transpose() *
                                                matrix_33);
  cout << "Eigen vals: \n" << eigen_solver.eigenvalues() << endl;
  cout << "========================================" << endl;
  cout << "Eigen vectors: \n" << eigen_solver.eigenvectors() << endl;

  cout << "========================================" << endl;

  // solve equation
  // Ax = b
  Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN =
      MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
  matrix_NN = matrix_NN * matrix_NN.transpose();
  Matrix<double, MATRIX_SIZE, 1> v_Nd = MatrixXd::Random(MATRIX_SIZE, 1);

  clock_t time_stt = clock();
  // use inverse
  Matrix<double, MATRIX_SIZE, 1> x = matrix_NN.inverse() * v_Nd;
  cout << "time of normal inverse is "
       << 1000 * (clock() - time_stt) / (double)CLOCKS_PER_SEC << "ms" << endl;
  cout << "x = " << x.transpose() << endl;

  cout << "========================================" << endl;
  // use Qr disolve
  time_stt = clock();
  x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
  cout << "time of Qr decomposition is "
       << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;
  cout << "x = " << x.transpose() << endl;

  cout << "========================================" << endl;

  time_stt = clock();
  x = matrix_NN.ldlt().solve(v_Nd);
  cout << "time of ldlt decomposition is "
       << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;
  cout << "x = " << x.transpose() << endl;

  return 0;
}


