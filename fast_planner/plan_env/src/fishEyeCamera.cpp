#include "plan_env/fishEyeCamera.h"
#include <iostream>

using namespace fast_planner;

FishEyeCamera::FishEyeCamera(ros::NodeHandle& nh)
{
    nh.param("sdf_map/fisheye/fx", fx, -1.0);
    nh.param("sdf_map/fisheye/fy", fy, -1.0);
    nh.param("sdf_map/fisheye/cx", cx, -1.0);
    nh.param("sdf_map/fisheye/cy", cy, -1.0);
    nh.param("sdf_map/fisheye/alpha", alpha, -1.0);
    nh.param("sdf_map/fisheye/xi", xi, -1.0);
    nh.param("/fisheyeDepth/offset_x", offset_x, -1);
    nh.param("/fisheyeDepth/offset_y", offset_y, -1);
    nh.param("/fisheyeDepth/crop_w", crop_w, -1);
    nh.param("/fisheyeDepth/crop_h", crop_h, -1);
    nh.param("/fisheyeDepth/scale", scale, -1.0);
    // scale intrinsic parameters
    fx *= scale;
    fy *= scale;
    cx *= scale;
    cy *= scale;
    // set fisheye camera mask
    std::string mask_file;
    nh.getParam("/fisheyeDepth/mask_file", mask_file);
    setMask(mask_file);
}

Eigen::Vector3d FishEyeCamera::unprojectPoint(ros::NodeHandle& nh, int u, int v, double depth)
{
    double mx = (u - cx)/fx;
    double my = (v - cy)/fy;
    double mxMySqr = mx*mx + my*my;

    double num = 1 - alpha*alpha*mxMySqr;
    double den = alpha * sqrt(1 - (2*alpha - 1) * mxMySqr) + 1 - alpha;
    double mz = num/den;

    double D = mz*mz + (1-xi*xi)*mxMySqr;
    if (D > 1e-5)
    {
        double fact = (mz*xi + sqrt(D)) / (mz*mz + mxMySqr);
        if (fact - xi < 0.1) // if the points go to close to 180° things go crazy!
            return Eigen::Vector3d::Zero();

        double fact2;
        Eigen::Vector3d point;

        fact2 = depth;
        point(0) = depth * fact * mx;
        point(1) = depth * fact * my;
        point(2) = depth * (fact * mz - xi);

        return point;
    }

    return Eigen::Vector3d::Zero();
}

void FishEyeCamera::setMask(const std::string& mask_file)
{
    std::cout << "mask file name: " << mask_file << std::endl;
    cv::Mat mask_original = cv::imread(mask_file, cv::IMREAD_GRAYSCALE);
    int height_o = mask_original.rows;
    int width_o = mask_original.cols;
    std::cout << "Original mask, height=" << height_o << ", width=" << width_o << std::endl;
    cv::Rect rect(offset_x, offset_y, crop_w, crop_h);  
    int height = int(crop_h*scale);
    int width = int(crop_w*scale);
    cv::resize(mask_original(rect), mask, cv::Size(width, height));  // Size, height after width
}

cv::Mat FishEyeCamera::getMask()
{
    return mask;
}

double FishEyeCamera::getScale()
{
    return scale;
}


