//
//  cv.hpp
//  superimpose-heatmap
//
//  Created by Supannee Tanathong on 22/01/2025.
//

#include <string>

#include <opencv2/core.hpp>

int Process(const std::string& rgbPath, const std::string& propPath, const std::string& outputPath, double alpha, int threshold, int approach);
int Superimpose(const std::string& rgbImagePath, const std::string& propImagePath, const std::string& outputPath, double alpha, int threshold, int approach);
cv::Mat MergeByBlending(const cv::Mat& rgbImage, const cv::Mat& propImage, double alpha);
cv::Mat MergeByBlendingAfterThresholding(const cv::Mat& rgbImage, const cv::Mat& rawPropImage, double alpha, int threshold);
cv::Mat MergeBySwithingRGBAndGray(const cv::Mat& rgbImage, const cv::Mat& rawPropImage, int threshold);

