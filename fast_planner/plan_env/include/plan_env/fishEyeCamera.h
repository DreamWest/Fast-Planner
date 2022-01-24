#ifndef _FISHEYECAMERA_
#define _FISHEYECAMERA_

#include <Eigen/Dense>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

namespace fast_planner
{
    class FishEyeCamera
    {
        double alpha, xi, fx, fy, cx, cy;
        int offset_x, offset_y, crop_w, crop_h;
        double scale;
        cv::Mat mask;
    public:
        FishEyeCamera() = default;
        FishEyeCamera(ros::NodeHandle& nh);
        Eigen::Vector3d unprojectPoint(ros::NodeHandle& nh, int u, int v, double depth);
        cv::Mat getMask();
        void setMask(const std::string& mask_file);
        double getScale();
    };
}

#endif