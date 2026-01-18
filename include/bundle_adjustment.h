#pragma once
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <opencv2/opencv.hpp>

// Calculate the error for a single point
struct ReprojectionError {
    ReprojectionError(double observed_x, double observed_y, double focal_len, double pp_x, double pp_y)
        : observed_x(observed_x), observed_y(observed_y), focal(focal_len), ppx(pp_x), ppy(pp_y) {}

    template <typename T>
    bool operator()(const T* const camera_rotation, // Angle-axis vector
                    const T* const camera_translation, // Translation vector
                    const T* const point_3d, // 3D point in world space
                    T* residuals) const { // Error x, Error y
        
        // Rotate the point
        T p_rotated[3];
        ceres::AngleAxisRotatePoint(camera_rotation, point_3d, p_rotated);

        // Translate the point
        T p[3];
        p[0] = p_rotated[0] + camera_translation[0];
        p[1] = p_rotated[1] + camera_translation[1];
        p[2] = p_rotated[2] + camera_translation[2];

        // Project to 2D
        // x' = x / z, y' = y / z
        T xp = p[0] / p[2];
        T yp = p[1] / p[2];

        // Apply Camera Intrinsics
        T predicted_x = T(focal) * xp + T(ppx);
        T predicted_y = T(focal) * yp + T(ppy);

        // Calculate Residual (Difference between observed and predicted)
        residuals[0] = predicted_x - T(observed_x);
        residuals[1] = predicted_y - T(observed_y);

        return true;
    }

    // Hide complexity
    static ceres::CostFunction* Create(double obs_x, double obs_y, double focal, double ppx, double ppy) {
        return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 3, 3, 3>(
            new ReprojectionError(obs_x, obs_y, focal, ppx, ppy)));
    }

    double observed_x, observed_y;
    double focal, ppx, ppy;
};