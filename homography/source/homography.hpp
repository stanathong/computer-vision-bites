//
//  homography.hpp
//  homography
//
//  Created by Supannee Tanathong on 04/06/2024.
//

#pragma once

#include <optional>
#include <vector>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

constexpr size_t kMinRequiredPointsForHomography = 4;

struct Camera {
    struct DistortionCoeffs {
        double k1 = 0.;
        double k2 = 0.;
        double p1 = 0.;
        double p2 = 0.;
        DistortionCoeffs() {};
        DistortionCoeffs(const double& k1, const double& k2, const double& p1, const double& p2) :
            k1(k1), k2(k2), p1(p1), p2(p2) {}
    };
    
    double fx;
    double fy;
    double cx;
    double cy;
    DistortionCoeffs distortionCoeffs;

    Camera() {}; // Uncalibrated camera
    Camera(const double& fx, const double& fy, const double& cx, const double& cy) :
        fx(fx), fy(fy), cx(cx), cy(cy) {}
    
    Camera(const double& fx, const double& fy, const double& cx, const double& cy, 
           DistortionCoeffs distortionCoeffs) :
        fx(fx), fy(fy), cx(cx), cy(cy)
        {
            distortionCoeffs = distortionCoeffs;
        }
};

// DLT solution, the function requires an exact 4 points to estimate homography.
std::optional<Eigen::Matrix3d> EstimateHomographyDLT(const std::vector<Eigen::Vector2d>& sourcePoints,
                                                     const std::vector<Eigen::Vector2d>& destinationPoints,
                                                     std::optional<Camera> camera1,
                                                     std::optional<Camera> camera2);

cv::Mat CreateHomographyImage(const cv::Mat& sourceImage, const Eigen::Matrix3d& H,
                              size_t outputImageWidth, size_t outputImageHeight);

///////////// Implementation //////////

std::optional<Eigen::Matrix3d> EstimateHomographyDLT(const std::vector<Eigen::Vector2d>& sourcePoints,
                                                     const std::vector<Eigen::Vector2d>& destinationPoints,
                                                     std::optional<Camera> camera1,
                                                     std::optional<Camera> camera2) {
    if (camera1.has_value() || camera2.has_value()) {
        // TODO: Undistort image points
    } else {
        std::cout << "Estimate homography for uncalibrated camera..." << std::endl;
    }
    
    // TODO: Normalise image points
    
    // If the number of points provided are greater than 4, take the first 4 correspondances for computation.
    if (sourcePoints.size() < 4 || destinationPoints.size() < 4) {
        std::cout << "Error: The problem requires an exact 4 corresponding points" << std::endl;
        return {};
    }
    
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(8, 9);
    for (size_t p = 0; p < 4; ++p) {
        // First row
        A.block<1, 3>(p*2, 0) = sourcePoints[p].homogeneous();
        A.block<1, 3>(p*2, 3) = Eigen::Vector3d::Zero();
        A.block<1, 3>(p*2, 6) = -sourcePoints[p].homogeneous() * destinationPoints[p][0];
        // Second row
        A.block<1, 3>(p*2 + 1, 0) = Eigen::Vector3d::Zero();
        A.block<1, 3>(p*2 + 1, 3) = sourcePoints[p].homogeneous();
        A.block<1, 3>(p*2 + 1, 6) = -sourcePoints[p].homogeneous() * destinationPoints[p][1];
    }
    
    Eigen::JacobiSVD<Eigen::Matrix<double, 8, 9>> svd(A, Eigen::ComputeFullV);
    Eigen::VectorXd nullspace = svd.matrixV().col(8);
    Eigen::Map<const Eigen::Matrix3d> H(nullspace.data());

    return H.transpose() / nullspace(8);
}

// Create an output image from the source image given that they are related by homography
cv::Mat CreateHomographyImage(const cv::Mat& sourceImage, const Eigen::Matrix3d& H,
                              size_t outputImageWidth, size_t outputImageHeight) {
    auto IsInsideSourceImage = [](double x, double y, int imageWidth, int imageHeight) {
        return !(x < 0 || x >= imageWidth || y < 0 || y >= imageHeight);
    };
    
    Eigen::Matrix3d invH = H.inverse(); // Homography from output or destination to source
    cv::Mat mapX(outputImageHeight, outputImageWidth, CV_32F, cv::Scalar(-1.f));
    cv::Mat mapY(outputImageHeight, outputImageWidth, CV_32F, cv::Scalar(-1.f));
    for (int r = 0; r < outputImageHeight; ++r) {
        for (int c = 0; c < outputImageWidth; ++c) {
            Eigen::Vector3d coordOutput(c, r, 1.);
            Eigen::Vector3d coordSource = (invH * coordOutput);
            double xSource = coordSource(0)/coordSource(2);
            double ySource = coordSource(1)/coordSource(2);
            if (coordSource(2) > 0 && IsInsideSourceImage(xSource, ySource, sourceImage.cols, sourceImage.rows)) {
                mapX.at<float>(r,c) = xSource;
                mapY.at<float>(r,c) = ySource;
            }
        }
    }
    cv::Mat outputImage;
    cv::remap(sourceImage, outputImage, mapX, mapY, cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);
    return outputImage;
}