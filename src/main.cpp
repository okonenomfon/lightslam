#include <iostream>
#include <opencv2/opencv.hpp>
#include "visual_odometry.h"
#include <Eigen/Core>

int main() {
    // Setup Camera
    // Focal length (718.8560) = guess for a generic webcam
    // Calibrate specific camera for better results
    double focal = 718.8560;
    cv::Point2d pp(607.1928, 185.2157);         // Center of image roughly

    VisualOdometry vo(focal, pp);

    // Open Webcam - Use '0' for webcam, or path to video file like "video.mp4"
    cv::VideoCapture cap(0); 
    if (!cap.isOpened()) {
        std::cerr << "Error: Camera not accessible" << std::endl;
        return -1;
    }

    // Create visualization map
    cv::Mat trajectory = cv::Mat::zeros(600, 600, CV_8UC3);
    int center_x = 300;
    int center_y = 500;

    std::cout << "Starting Visual Odometry... Move camera slowly." << std::endl;

    while (true) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) break;

        // --- VISUAL ODOMETRY PROCESSING ---
        // Process the image and calculates movement.
        vo.addFrame(frame);

        Eigen::Vector3d cur_t = vo.getCameraTranslation();
        
        // --- VISUALIZATION ---
        // Draw the current frame with tracked points
        cv::Mat out_frame = frame.clone();
        std::vector<cv::Point2f> points = vo.getTrackedPoints();
        for (const auto& p : points) {
            cv::circle(out_frame, p, 3, cv::Scalar(0, 255, 0), -1);
        }
        cv::imshow("Camera View", out_frame);

        // Draw the Map
        int draw_x = int(cur_t(0)) + center_x;
        int draw_y = int(cur_t(2)) + center_y;

        cv::circle(trajectory, cv::Point(draw_x, draw_y), 1, cv::Scalar(0, 0, 255), 2);
        cv::rectangle(trajectory, cv::Point(10, 30), cv::Point(550, 60), CV_RGB(0,0,0), -1);
        
        // Display Coordinates text
        char text[100];
        sprintf(text, "Coordinates: x=%.2fm z=%.2fm", cur_t(0), cur_t(2));
        cv::putText(trajectory, text, cv::Point(20, 50), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1);

        cv::imshow("Trajectory (Map)", trajectory);

        if (cv::waitKey(1) == 27) break; // ESC to quit
    }

    return 0;
}