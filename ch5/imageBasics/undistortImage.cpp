#include <string>
#include <opencv2/opencv.hpp>

using namespace std;

string image_file = "../Distorted Image from Slambook2.png";

int main(int argc, char** argv) {
    // Intrinsic parameters
    double fx = 458.654;
    double fy = 457.296;
    double cx = 367.215;
    double cy = 248.375;

    // Distortion parameters
    double k1 = -0.2834081;
    double k2 = 0.07395907;
    double p1 = 0.00019359;
    double p2 = 1.76187114e-05;

    cv::Mat image = cv::imread(image_file, 0);
    int rows = image.rows;
    int cols = image.cols;
    cv::Mat image_undistort = cv::Mat(rows, cols, CV_8UC1);

    // Image undistortion
    for (int v = 0; v < rows; ++v) {
        for (int u = 0; u < cols; ++u) {
            double x = (u - cx) / fx;
            double y = (v - cy) / fy;
            double r = sqrt(x * x + y * y);
            double x_dis = x * (1 + k1 * r * r + k2 * r * r * r * r) + 2 * p1 * x * y + p2 * (r * r + 2 * x * x);
            double y_dis = y * (1 + k1 * r * r + k2 * r * r * r * r) + p1 * (r * r + 2 * y * y) + 2 * p2 * x * y;
            double u_dis = fx * x_dis + cx;
            double v_dis = fy * y_dis + cy;

            // Assign value (nearest neighbor interpolation)
            if (u_dis >= 0 && v_dis >= 0 && u_dis < cols && v_dis < rows) {
                image_undistort.at<uchar>(v, u) = image.at<uchar>((int) v_dis, (int) u_dis);
            } else {
                image_undistort.at<uchar>(v, u) = 0;
            }
        }
    }

    cv::imshow("distort", image);
    cv::waitKey(0);
    cv::imshow("undistort", image_undistort);
    cv::waitKey(0);

    return 0;
}
