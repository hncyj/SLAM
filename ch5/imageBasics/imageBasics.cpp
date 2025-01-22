#include <iostream>
#include <chrono>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/core/core.hpp>

using namespace std;

int main(int argc, char **argv) {
    // Read the image specified by argv[1]
    cv::Mat image;
    image = cv::imread(argv[1]); // cv::imread function reads the image at the specified path

    // Check if the image file was read correctly
    if (image.data == nullptr) { // Data does not exist, possibly the file does not exist
        cerr << "File " << argv[1] << " does not exist." << endl;
        return 0;
    }

    // File read successfully, first output some basic information
    cout << "Image width: " << image.cols << ", height: " << image.rows << ", number of channels: " << image.channels() << endl;
    cv::imshow("image", image);
    cv::waitKey(0);

    // Check the type of the image
    if (image.type() != CV_8UC1 && image.type() != CV_8UC3) {
        // Image type does not meet the requirements
        cout << "Please input a color image or a grayscale image." << endl;
        return 0;
    }

    // Split image channels
    // std::vector<cv::Mat> channels;
    // cv::split(image, channels);
    
    // cv::Mat blueChannel, greenChannel, redChannel;
    // blueChannel = cv::Mat::zeros(image.size(), CV_8UC3);
    // greenChannel = cv::Mat::zeros(image.size(), CV_8UC3);
    // redChannel = cv::Mat::zeros(image.size(), CV_8UC3);

    // // Copy single channel to color image
    // cv::Mat blue[] = {channels[0], cv::Mat::zeros(image.size(), CV_8UC1), cv::Mat::zeros(image.size(), CV_8UC1)};
    // cv::merge(blue, 3, blueChannel);

    // cv::Mat green[] = {cv::Mat::zeros(image.size(), CV_8UC1), channels[1], cv::Mat::zeros(image.size(), CV_8UC1)};
    // cv::merge(green, 3, greenChannel);

    // cv::Mat red[] = {cv::Mat::zeros(image.size(), CV_8UC1), cv::Mat::zeros(image.size(), CV_8UC1), channels[2]};
    // cv::merge(red, 3, redChannel);

    // // Display each channel
    // cv::imshow("Blue channel (color display)", blueChannel);
    // cv::waitKey(0);
    // cv::imshow("Green channel (color display)", greenChannel);
    // cv::waitKey(0);
    // cv::imshow("Red channel (color display)", redChannel);
    // cv::waitKey(0);

    // Traverse the image, note that the following traversal method can also be used for random pixel access
    // Use std::chrono to time the algorithm
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    for (size_t y = 0; y < image.rows; y++) {
        // Use cv::Mat::ptr to get the row pointer of the image
        unsigned char *row_ptr = image.ptr<unsigned char>(y);  // row_ptr is the head pointer of the y-th row
        for (size_t x = 0; x < image.cols; x++) {
            // Access the pixel at (x, y)
            unsigned char *data_ptr = &row_ptr[x * image.channels()]; // data_ptr points to the pixel data to be accessed
            // Output each channel of the pixel, if it is a grayscale image, there is only one channel
            for (int c = 0; c != image.channels(); c++) {
                unsigned char data = data_ptr[c]; // data is the value of the c-th channel of I(x, y)
            }
        }
    }
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "Time used to traverse the image: " << time_used.count() << " seconds." << endl;

    // About cv::Mat copy
    // Direct assignment does not copy data
    // inborn reference type
    cv::Mat image_another = image;
    // Modifying image_another will cause image to change
    image_another(cv::Rect(0, 0, 100, 100)).setTo(0); // Set the top-left 100*100 block to zero
    cv::imshow("image", image);
    cv::waitKey(0);

    // Use the clone function to copy data
    cv::Mat image_clone = image.clone();
    image_clone(cv::Rect(0, 0, 100, 100)).setTo(255);
    cv::imshow("image", image);
    cv::imshow("image_clone", image_clone);
    cv::waitKey(0);

    // There are many basic operations for images, such as cropping, rotating, scaling, etc. Due to space limitations, they will not be introduced one by one. Please refer to the OpenCV official documentation for the calling method of each function.
    cv::destroyAllWindows();
    return 0;
}
