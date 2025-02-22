/**
 * @file TrajectoryDemo.cpp
 * @author chenyinjie
 * @date 2024-10-19
 */

#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <vector>

using namespace std;
using namespace Eigen;

const std::string file_path = "../../examples/Trajectory.txt";

void DrawTrajectory(vector<Isometry3d, aligned_allocator<Isometry3d>>& poses) {
    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
        .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glLineWidth(2);

        for (size_t i = 0; i < poses.size() - 1; i++) {
            // 画每个位姿的三个坐标轴
            Vector3d Ow = poses[i].translation();
            Vector3d Xw = poses[i] * (0.1 * Vector3d(1, 0, 0));
            Vector3d Yw = poses[i] * (0.1 * Vector3d(0, 1, 0));
            Vector3d Zw = poses[i] * (0.1 * Vector3d(0, 0, 1));
            glBegin(GL_LINES);
            glColor3f(1.0, 0.0, 0.0);
            glVertex3d(Ow[0], Ow[1], Ow[2]);
            glVertex3d(Xw[0], Xw[1], Xw[2]);
            glColor3f(0.0, 1.0, 0.0);
            glVertex3d(Ow[0], Ow[1], Ow[2]);
            glVertex3d(Yw[0], Yw[1], Yw[2]);
            glColor3f(0.0, 0.0, 1.0);
            glVertex3d(Ow[0], Ow[1], Ow[2]);
            glVertex3d(Zw[0], Zw[1], Zw[2]);
            glEnd();
        }
        // 画出连线
        for (size_t i = 0; i < poses.size(); i++) {
            glColor3f(0.0, 0.0, 0.0);
            glBegin(GL_LINES);
            auto p1 = poses[i], p2 = poses[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        std::this_thread::sleep_for(std::chrono::seconds(5));   // sleep 5 ms
    }
}

int main() {
    // read trajectory: position and transformation
    vector<Isometry3d, aligned_allocator<Isometry3d>> poses;
    ifstream fin(file_path);
    if (!fin.is_open()) {
        cerr << "file open failed." << endl;
        return -1;
    }

    while (!fin.eof()) {
        double time, tx, ty, tz, qx, qy, qz, qs;
        fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qs;
        
        // 这部分和原代码不同
        Quaterniond q(qs, qx, qy, qz);
        Isometry3d Twr(q);
        Twr.pretranslate(Vector3d(tx, ty, tz));
        poses.push_back(Twr);
    }

    cout << "read total " << poses.size() << " pose entries" << endl;
    // draw trajectory in pangolin
    DrawTrajectory(poses);

    return 0;
}