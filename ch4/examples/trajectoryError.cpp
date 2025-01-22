/**
 * @file trajectoryError.cpp
 * @author chenyinjie
 * @date 2024-10-27
 */

#include <iostream>
#include <string>
#include <fstream>
#include <unistd.h>

#include <pangolin/pangolin.h>
#include <sophus/se3.hpp>

using namespace std;
using namespace Sophus;

const string estimated_file_path = "../../examples/Estimated.txt";
const string groundtruth_file_path = "../../examples/Groundtruth.txt";

using TrajectoryType = vector<SE3d, Eigen::aligned_allocator<SE3d>>;

TrajectoryType ReadFile(const string& path) {
    fstream fin(path);
    TrajectoryType vec;

    if (!fin.is_open()) {
        cerr << "Traectory file open failed." << endl;
        return vec;
    }

    while (!fin.eof()) {
        double time, tx, ty, tz, qx, qy, qz, qw;
        fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        Eigen::Vector3d t(tx, ty, tz);
        Eigen::Matrix3d R(Eigen::Quaterniond(qx, qy, qz, qw));
        SE3d T(R, t);
        vec.emplace_back(T);
    }

    return vec;
}

void DrawTrajectory(TrajectoryType& gt, TrajectoryType& esti) {
    pangolin::CreateWindowAndBind("Trajectory View", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
    .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
    .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < gt.size() - 1; i++) {
            glColor3f(0.0f, 0.0f, 1.0f);  // blue for ground truth
            glBegin(GL_LINES);
            auto p1 = gt[i], p2 = gt[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }

        for (size_t i = 0; i < esti.size() - 1; i++) {
            glColor3f(1.0f, 0.0f, 0.0f);  // red for estimated
            glBegin(GL_LINES);
            auto p1 = esti[i], p2 = esti[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}

int main(int argc, char** argv) {
    TrajectoryType estimated = ReadFile(estimated_file_path);
    TrajectoryType groundtruth = ReadFile(groundtruth_file_path);
    assert(!estimated.empty() && !groundtruth.empty());
    assert(estimated.size() == groundtruth.size());

    // ATE_all
    double rmse = 0;
    int n = estimated.size();
    for (int i = 0; i < n; ++i) {
        SE3d gt = groundtruth[i];
        SE3d esti = estimated[i];
        // .log() returns the Lie algebra representation of the SE3
        // .norm() returns the length of the vector
        double error = (gt.inverse() * esti).log().norm();
        rmse += error * error;
    }
    rmse /= n;
    rmse = sqrt(rmse);
    cout << "RMSE All: " << rmse << endl;

    // ATE_trans
    rmse = 0;
    for (int i = 0; i < n; ++i) {
        SE3d gt = groundtruth[i];
        SE3d esti = estimated[i];
        double error = (gt.inverse() * esti).translation().norm();
        rmse += error * error;
    }
    rmse /= n;
    rmse = sqrt(rmse);
    cout << "RMSE Trans: " << rmse << endl;

    DrawTrajectory(groundtruth, estimated);

    return 0;
}


