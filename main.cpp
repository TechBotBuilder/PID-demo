#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <stdio.h>
#include <tchar.h>
#include <string>
#include <cmath>
#include <iomanip>

#include <ratio>         // std::this_thread::sleep_until
#include <chrono>         // std::chrono::system_clock
#include <ctime>          // std::time_t, std::tm, std::localtime, std::mktime

#include "nxt.h"

#include <fstream>

using namespace std::chrono;
using namespace cv;
using namespace std;

//TrungTN: http://stackoverflow.com/questions/997946/how-to-get-current-time-and-date-in-c
///Returns dd-mm-yy.hh-mm-ss
const std::string currentDateTime() {
    time_t now = time(0);
    struct tm tstruct;
    char buf[80];
    tstruct = * localtime( & now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%d-%m-%y.%H-%M-%S", & tstruct);

    return buf;
}

int main(int argc, char ** argv) {

    const double cycletime = 0.5;

    VideoCapture cap(0); //capture the video from web cam

    if (!cap.isOpened()) // if not success, exit program
    {
        cout << "Cannot open the web cam" << endl;
        return -1;
    }
    //http://www.monobrick.dk/software/c-library/
    Connection * connection = new Bluetooth();
    Motor * leftMotor = new Motor(OUT_B, connection);
    Motor * rightMotor = new Motor(OUT_C, connection);
    connection - > connect(4);

    //http://docs.opencv.org/doc/tutorials/highgui/video-write/video-write.html
    int ex = static_cast < int > (cap.get(CV_CAP_PROP_FOURCC)); // Get Codec Type- Int form
    //cout << ex <<endl;
    char EXT[] = {
        (char)(ex & 0XFF),
        (char)((ex & 0XFF00) >> 8),
        (char)((ex & 0XFF0000) >> 16),
        (char)((ex & 0XFF000000) >> 24),
        0
    };
    //cout << EXT << endl;
    Size S = Size((int) cap.get(CV_CAP_PROP_FRAME_WIDTH), // Acquire input size
        (int) cap.get(CV_CAP_PROP_FRAME_HEIGHT));
    VideoWriter outputVideo;
    string file_time = currentDateTime();
    string file_name = file_time + ".video.avi";
    //cout << file_name <<endl;
    outputVideo.open(file_name, ex = -1, (1.0 / cycletime), S, true);
    if (!outputVideo.isOpened()) {
        cout << "Could not open the output video for write!" << endl;
        return -1;
    }
    ofstream data_file;
    file_name = file_time + ".data.txt";
    //cout << file_name<<endl;
    data_file.open(file_name, ios::app | ios::out);
    if (!data_file.is_open()) {
        cout << "Could not open the data file for write!" << endl;
        return -1;
    }
    data_file << "Time\tDegree Error\tPID Output\tP Term\tI Term\tD Term\tLeft Motor Output\tRight Motor Output\n";

    const double M_PI = 3.14159265358979323846;
    int ellipse_dialator = 5;
    int rLowH = 159;
    int rHighH = 238;
    int rLowS = 91;
    int rHighS = 255;
    int rLowV = 26;
    int rHighV = 128;
    int yLowH = 0;
    int yHighH = 83;
    int yLowS = 175;
    int yHighS = 255;
    int yLowV = 136;
    int yHighV = 255;
    int bLowH = 57;
    int bHighH = 125;
    int bLowS = 188;
    int bHighS = 255;
    int bLowV = 0;
    int bHighV = 103;

    int bLastX = 0;
    int bLastY = 0;
    int yLastX = 0;
    int yLastY = 0;
    int rLastX = 0;
    int rLastY = 0;

    float degree_pid_output;
    float distance_pid_output;
    float center_x = 0;
    float center_y = 0;

    float degree_current_error = 0;
    float degree_previous_error = 0;
    float degree_p_term = 0;
    int degree_p_coefficient = 300;
    float degree_i_term = 0;
    int degree_i_coefficient = 10;
    float degree_i_collected = 0;
    float degree_d_term = 0;
    int degree_d_coefficient = 90;

    float distance_current_error = 0;
    float distance_previous_error = 0;
    float distance_p_term = 0;
    int distance_p_coefficient = 0;
    float distance_i_term = 0;
    int distance_i_coefficient = 0;
    float distance_i_collected = 0;
    float distance_d_term = 0;
    int distance_d_coefficient = 0;

    double pixels_wide = cap.get(CV_CAP_PROP_FRAME_WIDTH);
    double pixels_high = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
    double max_distance = sqrtf(pow(pixels_high, 2) + pow(pixels_wide, 2));

    float elapsed_time = 0;

    namedWindow("Control", CV_WINDOW_FREERATIO); //create a window called "Control"
    //Create trackbars in "Control" window
    cvCreateTrackbar("Deg P Gain", "Control", & degree_p_coefficient, 400);
    cvCreateTrackbar("Deg I Gain", "Control", & degree_i_coefficient, 400);
    cvCreateTrackbar("Deg D Gain", "Control", & degree_d_coefficient, 400);
    /*cvCreateTrackbar("Dis P Gain", "Control", &distance_p_coefficient, 400);
    cvCreateTrackbar("Dis I Gain", "Control", &distance_i_coefficient, 400);
    cvCreateTrackbar("Dis D Gain", "Control", &distance_d_coefficient, 400);*/

    //enable to find the right color values
    /*cvCreateTrackbar("rHueL", "Control", &rLowH, 255);
    cvCreateTrackbar("rHueH", "Control", &rHighH, 255);
    cvCreateTrackbar("rValL", "Control", &rLowV, 255);
    cvCreateTrackbar("rValH", "Control", &rHighV, 255);
    cvCreateTrackbar("rSatL", "Control", &rLowS, 255);
    cvCreateTrackbar("rSatH", "Control", &rHighS, 255);

    cvCreateTrackbar("bHueL", "Control", &bLowH, 255);
    cvCreateTrackbar("bHueH", "Control", &bHighH, 255);
    cvCreateTrackbar("bValL", "Control", &bLowV, 255);
    cvCreateTrackbar("bValH", "Control", &bHighV, 255);
    cvCreateTrackbar("bSatL", "Control", &bLowS, 255);
    cvCreateTrackbar("bSatH", "Control", &bHighS, 255);

    cvCreateTrackbar("yHueL", "Control", &yLowH, 255);
    cvCreateTrackbar("yHueH", "Control", &yHighH, 255);
    cvCreateTrackbar("yValL", "Control", &yLowV, 255);
    cvCreateTrackbar("yValH", "Control", &yHighV, 255);
    cvCreateTrackbar("ySatL", "Control", &yLowS, 255);
    cvCreateTrackbar("ySatH", "Control", &yHighS, 255);*/

    while (true) {
        high_resolution_clock::time_point t1 = high_resolution_clock::now();

        Mat imgOriginal;

        bool bSuccess = cap.read(imgOriginal); // read a new frame from video

        if (!bSuccess) //if not success, break loop
        {
            cout << "Cannot read a frame from video stream" << endl;
            break;
        }

        Mat imgHSV;

        cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

        Mat rImgThresholded;
        Mat yImgThresholded;
        Mat bImgThresholded;

        inRange(imgHSV, Scalar(rLowH, rLowS, rLowV), Scalar(rHighH, rHighS, rHighV), rImgThresholded); //Threshold the image for r
        inRange(imgHSV, Scalar(bLowH, bLowS, bLowV), Scalar(bHighH, bHighS, bHighV), bImgThresholded); //Threshold the image for b
        inRange(imgHSV, Scalar(yLowH, yLowS, yLowV), Scalar(yHighH, yHighS, yHighV), yImgThresholded); //Threshold the image for y

        //morphological opening (remove small objects from the foreground)
        erode(rImgThresholded, rImgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(ellipse_dialator, ellipse_dialator)));
        dilate(rImgThresholded, rImgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(ellipse_dialator, ellipse_dialator)));
        //morphological closing (fill small holes in the foreground)
        dilate(rImgThresholded, rImgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(ellipse_dialator, ellipse_dialator)));
        erode(rImgThresholded, rImgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(ellipse_dialator, ellipse_dialator)));

        erode(bImgThresholded, bImgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(ellipse_dialator, ellipse_dialator)));
        dilate(bImgThresholded, bImgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(ellipse_dialator, ellipse_dialator)));
        dilate(bImgThresholded, bImgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(ellipse_dialator, ellipse_dialator)));
        erode(bImgThresholded, bImgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(ellipse_dialator, ellipse_dialator)));

        erode(yImgThresholded, yImgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(ellipse_dialator, ellipse_dialator)));
        dilate(yImgThresholded, yImgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(ellipse_dialator, ellipse_dialator)));
        dilate(yImgThresholded, yImgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(ellipse_dialator, ellipse_dialator)));
        erode(yImgThresholded, yImgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(ellipse_dialator, ellipse_dialator)));

        Moments rMoments = moments(rImgThresholded);
        double M01 = rMoments.m01;
        double M10 = rMoments.m10;
        double Area = rMoments.m00;
        bool rFound = true;
        if ((M10 / Area) < 10000) {
            rLastX = M10 / Area;
            rLastY = M01 / Area;
        } else {
            rFound = false;
        }
        Moments bMoments = moments(bImgThresholded);
        M01 = bMoments.m01;
        M10 = bMoments.m10;
        Area = bMoments.m00;
        bool bFound = true;
        if ((M10 / Area) < 10000) {
            bLastX = M10 / Area;
            bLastY = M01 / Area;
        } else {
            bFound = false;
        }
        Moments yMoments = moments(yImgThresholded);
        M01 = yMoments.m01;
        M10 = yMoments.m10;
        Area = yMoments.m00;
        bool yFound = true;
        if ((M10 / Area) < 10000) {
            yLastX = M10 / Area;
            yLastY = M01 / Area;
        } else {
            yFound = false;
        }

        //PID SECTION
        float deg_p_c = degree_p_coefficient / 100.0;
        float deg_i_c = degree_i_coefficient / 100.0;
        float deg_d_c = degree_d_coefficient / 100.0;
        float dis_p_c = distance_p_coefficient / 100.0;
        float dis_i_c = distance_i_coefficient / 100.0;
        float dis_d_c = distance_d_coefficient / 100.0;
        center_x = (bLastX + yLastX) / 2;
        center_y = (bLastY + yLastY) / 2;

        //Much thinking and some trig help:http://keisan.casio.com/exec/system/1223522781; http://stackoverflow.com/questions/3162643/proper-trigonometry-for-rotating-a-point-around-the-origin
        float theta = atan2f((rLastY - center_y), (rLastX - center_x)); // sinf/cosf wants radian
        float transformed_x = (yLastX - center_x) * cosf(theta) + (yLastY - center_y) * sinf(theta);
        float transformed_y = -(yLastX - center_x) * sinf(theta) + (yLastY - center_y) * cosf(theta);

        //Formula: output = kp*e + ki*sum(e) + kd*recentslope(e)
        degree_current_error = atan2f(transformed_y, transformed_x) * 180 / M_PI;
        degree_p_term = deg_p_c * degree_current_error;
        degree_i_collected = degree_i_collected + (degree_current_error / cycletime);
        degree_i_term = deg_i_c * degree_i_collected;
        degree_d_term = deg_d_c * ((degree_current_error - degree_previous_error) / cycletime);
        degree_pid_output = degree_p_term + degree_i_term + degree_d_term;
        degree_previous_error = degree_current_error;

        distance_current_error = sqrtf(pow((center_x - rLastX), 2) + pow((center_y - rLastY), 2));
        distance_p_term = dis_p_c * distance_current_error;
        distance_i_collected = distance_i_collected + (distance_current_error / cycletime);
        distance_i_term = dis_i_c * distance_i_collected;
        distance_d_term = dis_d_c * ((distance_current_error - distance_previous_error) / cycletime);
        distance_pid_output = distance_p_term + distance_i_term + distance_d_term;
        distance_previous_error = distance_current_error;
        //END PID SECTION

        imshow("Red Thresholded Image", rImgThresholded); //show the thresholded image
        imshow("Blue Thresholded Image", bImgThresholded); //show the thresholded image
        imshow("Yellow Thresholded Image", yImgThresholded); //show the thresholded image
        imshow("Original", imgOriginal); //show the original image

        float final_distance_output = 100.0 * (distance_pid_output / max_distance); // BETWEEN 0 AND 100 %
        float final_degrees_output = 100.0 * (degree_pid_output / 180.0); //BETWEEN -100 AND 100%
        float left_output = (final_distance_output + final_degrees_output) / 2.0; // -100 TO 100%
        float right_output = (final_distance_output - final_degrees_output) / 2.0; // -100 TO 100%
        int out_right = round(0.2 * right_output);
        int out_left = round(0.2 * left_output);
        leftMotor - > on(out_left);
        rightMotor - > on(out_right);

        cout << "Distance: " << distance_current_error << endl;
        cout << "Bot:(" << center_x << "," << center_y << ")" << endl;
        cout << "Target:(" << rLastX << "," << rLastY << ")" << endl;
        cout << "PID out: " << distance_pid_output << endl;
        cout << "Terms: P:" << distance_p_term << "\tI:" << distance_i_term << "\tD:" << distance_d_term << endl;
        cout << "Coefficients: P:" << distance_p_coefficient << "\tI:" << distance_i_coefficient << "\tD:" << distance_d_coefficient << endl;
        cout << "Degree: " << degree_current_error << endl;
        cout << "PID out: " << degree_pid_output << endl;
        cout << "Terms: P:" << degree_p_term << "\tI:" << degree_i_term << "\tD:" << degree_d_term << endl;
        cout << "Coefficients: P:" << degree_p_coefficient << "\tI:" << degree_i_coefficient << "\tD:" << degree_d_coefficient << endl;
        cout << "L:" << out_left << " R:" << out_right << " Found:R:" << rFound << " B:" << bFound << " Y:" << yFound << endl;
        cout << endl;

        data_file << elapsed_time << "\t" << degree_current_error << "\t" << degree_pid_output << "\t" << degree_p_term << "\t" << degree_i_term << "\t" << degree_d_term << "\t" << out_left << "\t" << out_right << "\n";
        //http://feelmare.blogspot.com/2014/03/opencv-study-write-text-on-image.html
        char TestStr[128];
        sprintf(TestStr, "Elapsed Time:%5.2f", elapsed_time);
        putText(imgOriginal, TestStr, Point(10, 110), FONT_HERSHEY_PLAIN, 1, Scalar(20, 20, 20), 0.2, 1, false); //OutImg is Mat class;
        sprintf(TestStr, "Bot:(%f,%f)  Target:(%d,%d)", center_x, center_y, rLastX, rLastY);
        putText(imgOriginal, TestStr, Point(10, 90), FONT_HERSHEY_PLAIN, 1, Scalar(20, 20, 20), 0.2, 1, false); //OutImg is Mat class;
        sprintf(TestStr, "Distance:%5.2f  DegreeError:%5.2f", distance_current_error, degree_current_error);
        putText(imgOriginal, TestStr, Point(10, 70), FONT_HERSHEY_PLAIN, 1, Scalar(20, 20, 20), 0.2, 1, false); //OutImg is Mat class;
        sprintf(TestStr, "Degree PID Output:%5.2f  PTerm:%5.2f  ITerm:%5.2f  DTerm:%5.2f", degree_pid_output, degree_p_term, degree_i_term, degree_d_term);
        putText(imgOriginal, TestStr, Point(10, 50), FONT_HERSHEY_PLAIN, 1, Scalar(20, 20, 20), 0.2, 1, false); //OutImg is Mat class;
        sprintf(TestStr, "LeftOutput:%+d  RightOutput:%+d", out_left, out_right);
        putText(imgOriginal, TestStr, Point(10, 30), FONT_HERSHEY_PLAIN, 1, Scalar(20, 20, 20), 0.2, 1, false); //OutImg is Mat class;

        outputVideo << imgOriginal;

        if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop.
        //Also, bluetooth wants 30ms between commands, so this guarentees it.
        {
            cout << "esc key is pressed by user" << endl;
            break;
        }
        elapsed_time += cycletime;
        bool stillkeepwaiting = true;
        while (stillkeepwaiting) {
            high_resolution_clock::time_point t2 = high_resolution_clock::now();
            duration < double > timespan = duration_cast < duration < double >> (t2 - t1);
            if (timespan.count() > cycletime) {
                stillkeepwaiting = false;
            }
        }
    }
    leftMotor - > stop();
    rightMotor - > stop();
    connection - > disconnect();
    outputVideo.release();
    data_file.close();
    return 0;

}
