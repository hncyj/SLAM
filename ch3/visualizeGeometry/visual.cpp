/**
 * @file visual.cpp
 * @author chenyinjie
 * @date 2024-10-18
 */

#include <iomanip>
#include <iostream>

#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

struct RotationMatrix {
  Matrix3d matrix = Matrix3d::Identity();
};

// 设置输出格式
ostream& operator<<(ostream& out, const RotationMatrix& rotation_matrix) {
  out.setf(ios::fixed);
  Matrix3d matrix = rotation_matrix.matrix;
  out << "=";
  out << setprecision(2) << "[" << matrix(0, 0) << "," << matrix(0, 1) << ","
      << matrix(0, 2) << "],"
      << "[" << matrix(1, 0) << "," << matrix(1, 1) << "," << matrix(1, 2)
      << "],"
      << "[" << matrix(2, 0) << "," << matrix(2, 1) << "," << matrix(2, 2)
      << "]";

  return out;
}

istream& operator>>(istream& in, RotationMatrix& rotation_matrix) {
  return in;
}

struct TranslationVector {
  Vector3d trans = Vector3d(0, 0, 0);
};

ostream& operator<<(ostream& out, const TranslationVector& translation_vector) {
  out << "= [" << translation_vector.trans(0) << ", "
      << translation_vector.trans(1) << ", " << translation_vector.trans(2)
      << "]";
  return out;
}

istream& operator>>(istream& in, TranslationVector& translation_vector) {
  return in;
}

struct QuaternionDraw {
  Quaterniond q;
};

ostream& operator<<(ostream& out, const QuaternionDraw quat) {
  auto c = quat.q.coeffs();
  out << "= [" << c[0] << ", " << c[1] << ", " << c[2] << ", " << c[3] << "]";
  return out;
}

istream& operator>>(istream& in, const QuaternionDraw quat) {
  return in;
}

int main() {
  pangolin::CreateWindowAndBind("visualize geometry", 1000, 600);
  glEnable(GL_DEPTH_TEST);
  pangolin::OpenGlRenderState s_cam(
    pangolin::ProjectionMatrix(1000, 600, 420, 420, 500, 300, 0.1, 1000),
    pangolin::ModelViewLookAt(3, 3, 3, 0, 0, 0, pangolin::AxisY)
    );

  const int UI_WIDTH = 500;

  pangolin::View& d_cam = pangolin::CreateDisplay().SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0, -1000.0f / 600.0f).SetHandler(new pangolin::Handler3D(s_cam));

  // ui
  pangolin::Var<RotationMatrix> rotation_matrix("ui.R", RotationMatrix());
  pangolin::Var<TranslationVector> translation_vector("ui.t", TranslationVector());
  pangolin::Var<TranslationVector> euler_angles("ui.rpy", TranslationVector());
  pangolin::Var<QuaternionDraw> quaternion("ui.q", QuaternionDraw());
  pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));

  while (!pangolin::ShouldQuit()) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    d_cam.Activate(s_cam);

    pangolin::OpenGlMatrix matrix = s_cam.GetModelViewMatrix();
    Matrix<double, 4, 4> m;
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        m(i, j) = matrix.m[4 * i + j];  // OpenGlMatrix 是列优先存储
      }
    }
    // m = m.inverse();
    RotationMatrix R;
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        R.matrix(i, j) = m(j, i);
    rotation_matrix = R;

    TranslationVector t;
    t.trans = Vector3d(m(0, 3), m(1, 3), m(2, 3));
    t.trans = -R.matrix * t.trans;
    translation_vector = t;

    TranslationVector euler;
    euler.trans = R.matrix.transpose().eulerAngles(2, 1, 0);
    euler_angles = euler;

    QuaternionDraw quat;
    quat.q = Quaterniond(R.matrix);
    quaternion = quat;

    glColor3f(1.0, 1.0, 1.0);

    pangolin::glDrawColouredCube();
    // draw the original axis
    glLineWidth(3);
    glColor3f(0.8f, 0.f, 0.f);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(10, 0, 0);
    glColor3f(0.f, 0.8f, 0.f);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 10, 0);
    glColor3f(0.2f, 0.2f, 1.f);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 0, 10);
    glEnd();

    pangolin::FinishFrame();
  }

  return 0;
}
