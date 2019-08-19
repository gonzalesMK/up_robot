#include "ros/ros.h"
#include <iostream>
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>

using namespace std;
using namespace cv;

Mat src;
Mat src_gray;
int thresh = 200;
int max_thresh = 255;
int max_radius = 500;
int max_translation = 500;
int param1 = 100;
int param2 = 300;
int max_param1 = 500;
int max_param2 = 500;

RNG rng(12345);

int R = 1300;
int T = 1700;
void thresh_callback(int, void *);
void radius_callback(int, void *);
void translation_callback(int, void *);
void do_contour();
void do_thresh(int, void *);

void param1_cb(int, void *);
void param2_cb(int, void *);
void on_low_r_thresh_trackbar(int, void *);
void on_high_r_thresh_trackbar(int, void *);
void on_low_g_thresh_trackbar(int, void *);
void on_high_g_thresh_trackbar(int, void *);
void on_low_b_thresh_trackbar(int, void *);
void on_high_b_thresh_trackbar(int, void *);

int low_r = 30, low_g = 30, low_b = 98;
int high_r = 100, high_g = 100, high_b = 255;

int morph_elem = 0;
int morph_size = 0;
int morph_operator = 0;
int const max_operator = 4;
int const max_elem = 2;
int const max_kernel_size = 21;

Mat thresh_output;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "cv_example");

    String imageName("/home/gonzales/Documents/projetos_ws/src/projetos/img2.jpg"); // by default

    if (argc > 1)
    {
        imageName = argv[1];
    }
    src = imread(imageName, IMREAD_COLOR);
    if (src.empty())
    {
        cerr << "No image supplied ..." << endl;
        return -1;
    }
    cvtColor(src, src_gray, COLOR_BGR2GRAY);
    blur(src_gray, src_gray, Size(3, 3));
    const char *source_window = "Source";

    namedWindow(source_window, WINDOW_NORMAL);
    resizeWindow(source_window, 600, 600);
    imshow(source_window, src);

    max_radius = src.cols;
    max_translation = src.cols;
    thresh_output = src_gray.clone();

    createTrackbar("Low R", "Source", &low_r, 255, do_thresh);
    createTrackbar("High R", "Source", &high_r, 255, do_thresh);
    createTrackbar("Low G", "Source", &low_g, 255, do_thresh);
    createTrackbar("High G", "Source", &high_g, 255, do_thresh);
    createTrackbar("Low B", "Source", &low_b, 255, do_thresh);
    createTrackbar("High B", "Source", &high_b, 255, do_thresh);

    createTrackbar(" Param 1", "Source", &param1, max_param1, do_thresh);
    createTrackbar(" Param 2", "Source", &param2, max_param1, do_thresh);

    //createTrackbar(" Canny thresh:", "Source", &thresh, max_thresh, thresh_callback);
    createTrackbar(" Radius:", "Source", &R, max_radius, radius_callback);
    createTrackbar(" T:", "Source", &T, max_translation, translation_callback);

    createTrackbar("Operator:\n 0: Opening - 1: Closing  \n 2: Gradient - 3: Top Hat \n 4: Black Hat", "Source", &morph_operator, max_operator, do_thresh);

    createTrackbar("Element:\n 0: Rect - 1: Cross - 2: Ellipse", "Source",
                   &morph_elem, max_elem,
                   do_thresh);
    createTrackbar("Kernel size:\n 2n +1", "Source",
                   &morph_size, max_kernel_size,
                   do_thresh);

    thresh_callback(0, 0);

    namedWindow("Contours", WINDOW_NORMAL);
    resizeWindow("Contours", 600, 600);

    namedWindow("Threshold", WINDOW_NORMAL);
    resizeWindow("Threshold", 600, 600);

    waitKey(0);

    return (0);
}

void translation_callback(int, void *)
{
    do_contour();
}
void radius_callback(int, void *)
{
    do_contour();
    /*
    Mat drawing = Mat::zeros(src_gray.size(), CV_8UC3);
    Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));

    Point pt1, pt2;

    pt1.x = center;
    pt1.y = src_gray.rows / 2;
    pt2.x = R + center;
    pt2.y = src_gray.rows / 2;

    line(drawing, pt1, pt2, color, 10);

    imshow("Contours", drawing);
     */
}

void thresh_callback(int, void *)
{
    do_contour();
}
/*
void param1_cb(int, void *)
{
    do_thresh();
}
void param2_cb(int, void *)
{
    do_thresh();
}
*/
void do_thresh(int, void *)
{
    // Threshold image to make circle more visible

    inRange(src_gray, Scalar(low_b, low_g, low_r), Scalar(high_b, high_g, high_r), thresh_output);

    threshold(thresh_output, thresh_output, 100, 255, THRESH_BINARY_INV);

    int operation = morph_operator + 2;
    Mat element = getStructuringElement(2, Size(2 * morph_size + 1, 2 * morph_size + 1), Point(morph_size, morph_size));
    morphologyEx(thresh_output, thresh_output, operation, element);

    /* This is for HoughCirlces 
  vector<Vec3f> circles;

  HoughCircles(thresh_output, circles, HOUGH_GRADIENT, 5,
                 thresh_output.rows / 16, // change this value to detect circles with different distances to each other
                 param1, param2, 0, 0     // change the last two parameters
                                          // (min_radius & max_radius) to detect larger circles
    );

    ROS_INFO_STREAM("found " << (circles.size()) << " circles");
    for (size_t i = 0; i < circles.size(); i++)
    {
        Vec3i c = circles[i];
        Point center = Point(c[0], c[1]);
        // circle center
        circle(thresh_output, center, 1, Scalar(0, 100, 100), 10, LINE_AA);
        // circle outline
        int radius = c[2];
        circle(thresh_output, center, radius, Scalar(100, 0, 100), 10, LINE_AA);
    }
*/
    namedWindow("Threshold", WINDOW_NORMAL);
    resizeWindow("Threshold", 600, 600);
    imshow("Threshold", thresh_output);
}

void do_contour()
{
    Mat canny_output;
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    Canny(thresh_output, canny_output, thresh, thresh * 2, 3);
    findContours(canny_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

    int center = T;

    // Crop Contours outside cylinder

    for (auto it = contours.begin(); it != contours.end(); it++)
    {

        for (auto j = (*it).begin(); j != (*it).end(); j++)
        {
            int L = (*j).x - (center);

            if (abs(L) > (R))
            {
                (*it).erase(j--);
            }
        }

        if ((*it).size() == 0)
        {
            contours.erase(it--);
        }
    }

    // Delete Empty Contours

    // Original Countours
    Mat drawing = Mat::zeros(canny_output.size(), canny_output.type()); //thresh_output.clone();//Mat::zeros(canny_output.size(), CV_8UC3); //src.clone();//
    for (size_t i = 0; i < contours.size(); i++)
    {
        Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        drawContours(drawing, contours, (int)i, color, 2, 8, hierarchy, 0, Point());
    }

    // Transform X coordinates of contours: cylinder transform
    for (auto it = contours.begin(); it != contours.end(); it++)
    {

        for (auto j = (*it).begin(); j != (*it).end(); j++)
        {

            double L = ((*j).x - center);

            double theta = asin((double)L / R);

            (*j).x = R * theta + center;
        }
    }

    // Draw new contours
    for (size_t i = 0; i < contours.size(); i++)
    {
        Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        drawContours(drawing, contours, (int)i, color, 2, 8, hierarchy, 0, Point());
    }

    Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
    /*
    vector<Vec3f> circles;

    HoughCircles(drawing, circles, HOUGH_GRADIENT, 5,
                 drawing.rows / 2,          // change this value to detect circles with different distances to each other
                 param1, param2, 1000, 2500 // change the last two parameters
                                            // (min_radius & max_radius) to detect larger circles
    );

    ROS_INFO_STREAM("found " << (circles.size()) << " circles");
    for (size_t i = 0; i < circles.size(); i++)
    {
        Vec3i c = circles[i];
        Point center = Point(c[0], c[1]);
        // circle center
        circle(drawing, center, 1, Scalar(0, 100, 100), 10, LINE_AA);
        // circle outline
        int radius = c[2];
        circle(drawing, center, radius, Scalar(100, 0, 100), 10, LINE_AA);
    }
*/

    vector<vector<Point>> contours_poly(contours.size());
    vector<Rect> boundRect(contours.size());
    vector<Point2f> Center(contours.size());
    vector<float> Radius(contours.size());
    for (size_t i = 0; i < contours.size(); i++)
    {
        approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
        boundRect[i] = boundingRect(Mat(contours_poly[i]));
        minEnclosingCircle(contours_poly[i], Center[i], Radius[i]);
    }
    //Mat drawing = Mat::zeros(thresh_output.size(), CV_8UC3);
    for (size_t i = 0; i < contours.size(); i++)
    {
        Scalar color = Scalar(100, 100, 100);
        drawContours(drawing, contours_poly, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point());
        rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0);
        circle(drawing, Center[i], (int)Radius[i], color, 5, 8, 0);
    }

    Point pt1, pt2;
    pt1.x = -R + center;
    pt1.y = src_gray.rows / 2;
    pt2.x = R + center;
    pt2.y = src_gray.rows / 2;

    line(drawing, pt1, pt2, color, 10);

    pt1.x = -R * 3.1415 / 2 + center;
    pt1.y = src_gray.rows / 2 + 100;
    pt2.x = R * 3.1415 / 2 + center;
    pt2.y = src_gray.rows / 2 + 100;

    line(drawing, pt1, pt2, color, 10);

    float max = 0;
    for (int j = 0; j < Radius.size(); j++)
    {   
        max = Radius[j] > max ? Radius[j] : max;
        ROS_INFO_STREAM("Radius Values" <<j << " : "<< Radius[j]);
    }
    namedWindow("Contours", WINDOW_NORMAL);
    resizeWindow("Contours", 600, 600);

    ROS_INFO_STREAM("FINAL CIRCLE RADIUS: " << max);
    imshow("Contours", drawing);
}
/*
void on_low_r_thresh_trackbar(int, void *)
{
    low_r = min(high_r - 1, low_r);
    do_thresh();
    setTrackbarPos("Low R", "Threshold", low_r);
}
void on_high_r_thresh_trackbar(int, void *)
{
    high_r = max(high_r, low_r + 1);
    do_thresh();
    setTrackbarPos("High R", "Threshold", high_r);
}
void on_low_g_thresh_trackbar(int, void *)
{
    low_g = min(high_g - 1, low_g);
    do_thresh();
    setTrackbarPos("Low G", "Threshold", low_g);
}
void on_high_g_thresh_trackbar(int, void *)
{
    high_g = max(high_g, low_g + 1);
    do_thresh();
    setTrackbarPos("High G", "Threshold", high_g);
}
void on_low_b_thresh_trackbar(int, void *)
{
    low_b = min(high_b - 1, low_b);
    do_thresh();
    setTrackbarPos("Low B", "Threshold", low_b);
}
void on_high_b_thresh_trackbar(int, void *)
{
    high_b = max(high_b, low_b + 1);
    do_thresh();
    setTrackbarPos("High B", "Threshold", high_b);
}
*/