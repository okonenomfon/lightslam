#include "visual_odometry.h"
#include "bundle_adjustment.h"
#include <opencv2/calib3d.hpp>
#include <iostream>

VisualOdometry::VisualOdometry(double f, cv::Point2d pp) {
    focal_length = f;
    principal_point = pp;
    R_f = Eigen::Matrix3d::Identity();
    t_f = Eigen::Vector3d::Zero();
}

Eigen::Matrix3d VisualOdometry::cv2eigenRot(const cv::Mat& R) {
    Eigen::Matrix3d R_e;
    for(int i=0; i<3; i++) for(int j=0; j<3; j++) R_e(i,j) = R.at<double>(i,j);
    return R_e;
}
Eigen::Vector3d VisualOdometry::cv2eigenTrans(const cv::Mat& t) {
    return Eigen::Vector3d(t.at<double>(0), t.at<double>(1), t.at<double>(2));
}

void VisualOdometry::detectNewFeatures(const cv::Mat& img_gray) {
    std::vector<cv::Point2f> new_features;
    cv::goodFeaturesToTrack(img_gray, new_features, 3000, 0.01, 10);
    prev_points.insert(prev_points.end(), new_features.begin(), new_features.end());
}

void VisualOdometry::addFrame(const cv::Mat& frame) {
    cv::Mat cur_frame_gray;
    cv::cvtColor(frame, cur_frame_gray, cv::COLOR_BGR2GRAY);

    if (prev_frame_gray.empty()) {
        detectNewFeatures(cur_frame_gray);
        prev_frame_gray = cur_frame_gray.clone();
        return;
    }

    std::vector<cv::Point2f> cur_points;
    std::vector<uchar> status;
    std::vector<float> err;
    if(!prev_points.empty())
        cv::calcOpticalFlowPyrLK(prev_frame_gray, cur_frame_gray, prev_points, cur_points, status, err);

    std::vector<cv::Point2f> good_prev, good_cur;
    for (size_t i = 0; i < status.size(); i++) {
        if (status[i] == 1) {
            good_prev.push_back(prev_points[i]);
            good_cur.push_back(cur_points[i]);
        }
    }

    if (good_prev.size() > 50) {
        cv::Mat E, R_cv, t_cv, mask;
        E = cv::findEssentialMat(good_cur, good_prev, focal_length, principal_point, cv::RANSAC, 0.999, 1.0, mask);
        cv::recoverPose(E, good_cur, good_prev, R_cv, t_cv, focal_length, principal_point, mask);

        cv::Mat points4D;
        cv::Mat P1 = cv::Mat::eye(3, 4, CV_64F); // Previous Pose
        cv::Mat P2 = cv::Mat::eye(3, 4, CV_64F); // Current Pose
        R_cv.copyTo(P2(cv::Rect(0,0,3,3)));
        t_cv.copyTo(P2(cv::Rect(3,0,1,3)));
        
        // P1 and P2 must be properly scaled with `focal_length` and `principal_point`
        cv::Mat K = (cv::Mat_<double>(3,3) << focal_length, 0, principal_point.x, 0, focal_length, principal_point.y, 0, 0, 1);
        cv::triangulatePoints(K*P1, K*P2, good_prev, good_cur, points4D);

        points4D.convertTo(points4D, CV_64F);

        // --- CERES OPTIMIZATION START ---
        // Convert initial guesses to double arrays for Ceres
        double rotation_vec[3]; // Angle-axis
        cv::Mat rvec;
        cv::Rodrigues(R_cv, rvec); // Convert Matrix to Vector
        rotation_vec[0] = rvec.at<double>(0);
        rotation_vec[1] = rvec.at<double>(1);
        rotation_vec[2] = rvec.at<double>(2);

        double translation_vec[3];
        translation_vec[0] = t_cv.at<double>(0);
        translation_vec[1] = t_cv.at<double>(1);
        translation_vec[2] = t_cv.at<double>(2);

        ceres::Problem problem;
        
        // Loop through all points and add residuals to the problem
        for (int i = 0; i < good_cur.size(); i++) {
            // Only use inliers from RANSAC
            if (mask.at<uchar>(i) == 0) continue; 

            // Convert Homogeneous 4D point to 3D
            double w = points4D.at<double>(3, i);
            double p3d[3];
            p3d[0] = points4D.at<double>(0, i) / w;
            p3d[1] = points4D.at<double>(1, i) / w;
            p3d[2] = points4D.at<double>(2, i) / w;

            // Add Block
            ceres::CostFunction* cost_function = ReprojectionError::Create(
                good_cur[i].x, good_cur[i].y, focal_length, principal_point.x, principal_point.y
            );
            
            // We hold the 3D point constant and optimize R and t (Motion Only BA)
            problem.AddResidualBlock(cost_function, new ceres::HuberLoss(1.0), rotation_vec, translation_vec, p3d);
            problem.SetParameterBlockConstant(p3d); // Don't move the points, just the camera
        }

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_SCHUR;
        options.minimizer_progress_to_stdout = false; 
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        
        // Convert back to Matrices
        cv::Mat rvec_optimized = (cv::Mat_<double>(3,1) << rotation_vec[0], rotation_vec[1], rotation_vec[2]);
        cv::Mat R_opt;
        cv::Rodrigues(rvec_optimized, R_opt);
        cv::Mat t_opt = (cv::Mat_<double>(3,1) << translation_vec[0], translation_vec[1], translation_vec[2]);
        
        // --- CERES OPTIMIZATION END ---

        // Update Global State (using Optimized values)
        double scale = 1.0;
        Eigen::Matrix3d R_e = cv2eigenRot(R_opt);
        Eigen::Vector3d t_e = cv2eigenTrans(t_opt);

        if ((scale > 0.1) && (t_e(2) > t_e(0)) && (t_e(2) > t_e(1))) {
            t_f = t_f + R_f * t_e * scale;
            R_f = R_f * R_e;
        }
    }

    // Replenish features
    if (good_prev.size() < 2000) {
        detectNewFeatures(cur_frame_gray);
        std::vector<cv::Point2f> combined = good_cur; 
        std::vector<cv::Point2f> new_feats;
        cv::goodFeaturesToTrack(cur_frame_gray, new_feats, 3000 - good_cur.size(), 0.01, 10);
        combined.insert(combined.end(), new_feats.begin(), new_feats.end());
        prev_points = combined;
    } else {
        prev_points = good_cur;
    }
    prev_frame_gray = cur_frame_gray.clone();
}

Eigen::Vector3d VisualOdometry::getCameraTranslation() {
    return t_f;
}

std::vector<cv::Point2f> VisualOdometry::getTrackedPoints() {
    return prev_points;
}