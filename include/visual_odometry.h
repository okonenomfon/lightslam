#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

class VisualOdometry {
public:
    VisualOdometry(double focal_length, cv::Point2d pp);
    void addFrame(const cv::Mat& frame);
    
    // Return Eigen vector
    Eigen::Vector3d getCameraTranslation();
    std::vector<cv::Point2f> getTrackedPoints();

private:
    double focal_length;
    cv::Point2d principal_point;

    // Current Camera Pose
    Eigen::Matrix3d R_f; // Rotation Matrix
    Eigen::Vector3d t_f; // Translation Vector

    cv::Mat prev_frame_gray;
    std::vector<cv::Point2f> prev_points;

    void detectNewFeatures(const cv::Mat& img_gray);
    
    // Convert OpenCV mat to Eigen
    Eigen::Matrix3d cv2eigenRot(const cv::Mat& R);
    Eigen::Vector3d cv2eigenTrans(const cv::Mat& t);
};