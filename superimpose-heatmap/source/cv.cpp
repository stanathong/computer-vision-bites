//
//  cv.cpp
//  superimpose-heatmap
//
//  Created by Supannee Tanathong on 23/01/2025.
//

#include <filesystem>
#include <vector>

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include "cv.hpp"
#include "utils.hpp"

namespace fs = std::filesystem;

const std::string kImageExtensions = ".png .jpg .jpeg";

int Process(const std::string& rgbPath, const std::string& propPath, const std::string& outputPath, double alpha, int threshold, int approach) {
    bool isDirectory = fs::is_directory(fs::path(rgbPath));
    if (!isDirectory) {
        // Process a single file
        return Superimpose(rgbPath, propPath, outputPath, alpha, threshold, approach);
    } else {
        // Process directory
        std::vector<int> success(2,0); // count the number of success and failure
        const fs::path rgbPathSystem{rgbPath};
        for (auto const& item : fs::directory_iterator{rgbPathSystem}) {
            std::cout << "Processing image " << item.path().string() << std::endl;
            std::string ext = item.path().extension().string();
            if (!ext.empty() && kImageExtensions.find(ext) != std::string::npos) {
                // Get its corresponding property file
                std::string propFilePath = propPath + '/' + item.path().filename().string();
                if (fs::exists(fs::path{propFilePath}) == false) {
                    continue;
                }
                int ret = Superimpose(item.path().string(), propFilePath, outputPath, alpha, threshold, approach);
                ret == 0 ? success[0]++ : success[1]++;
            }
        }
        std::cout << "Image processing " << success[0] << " success out of " << success[0] + success[1] << std::endl;
    }
    return 0;
}

int Superimpose(const std::string& rgbImagePath, const std::string& propImagePath, const std::string& outputPath, double alpha, int threshold, int approach) {
    try {
        cv::Mat rgbImage = cv::imread(rgbImagePath, cv::IMREAD_COLOR);
        cv::Mat propImage = cv::imread(propImagePath, cv::IMREAD_GRAYSCALE);
        if (rgbImage.empty() || propImage.empty()) {
            std::cout << "Invalid input image: " << rgbImagePath << std::endl;
            return -1;
        }

        cv::Mat outputImage;
        
        if (approach == 0) {
            outputImage = MergeByBlending(rgbImage, propImage, alpha);
        } else if (approach == 1) {
            outputImage = MergeByBlendingAfterThresholding(rgbImage, propImage, alpha, threshold);
        } else {
            outputImage = MergeBySwithingRGBAndGray(rgbImage, propImage, threshold);
        }

        // Write the output path to file
        auto [dir, name, ext] = ExtractFromFilePath(propImagePath);
        std::string outputFilename = outputPath + "/" + name + "." + ext;
        cv::imwrite(outputFilename, outputImage);
        
    } catch (const std::exception& e) {
        std::cout << "Error " << e.what() << std::endl;
        return -1;
    }
    return 0;
}

// Perform alpha-blending without filtering the propImage
cv::Mat MergeByBlending(const cv::Mat& rgbImage, const cv::Mat& propImage, double alpha) {
    // Convert propImage to heapmap
    cv::Mat heatmapImage;
    cv::applyColorMap(propImage, heatmapImage, cv::COLORMAP_JET);
    
    // Construct the output image
    cv::Mat outputImage(rgbImage.rows, rgbImage.cols, rgbImage.type());
    
    // Alpha-blending
    for (size_t r = 0; r < rgbImage.rows; ++r) {
        for (size_t c = 0; c < rgbImage.cols; ++c) {
            for (size_t chan = 0; chan < rgbImage.channels(); ++chan) {
                outputImage.at<cv::Vec3b>(r,c)[chan] =
                    cv::saturate_cast<uchar>((1.0 - alpha) * rgbImage.at<cv::Vec3b>(r,c)[chan] +
                                             alpha * heatmapImage.at<cv::Vec3b>(r,c)[chan]);
            }
        }
    }
    
    return outputImage;
}

// Perform alpha-blending only when the property pixel is greater than threshold
cv::Mat MergeByBlendingAfterThresholding(const cv::Mat& rgbImage, const cv::Mat& rawPropImage, double alpha, int threshold) {
    // Assign pixels whose value below threshold to 0.
    cv::Mat mask; // result in 1 channel image
    cv::threshold(rawPropImage, mask, threshold, 255, cv::THRESH_TOZERO);
    
    // Only pixels whose value greater than threshold are used in blending
    cv::Mat propImage;
    cv::cvtColor(mask, propImage, cv::COLOR_GRAY2BGR);
    
    // Convert propImage to heapmap
    cv::Mat heatmapImage;
    cv::applyColorMap(propImage, heatmapImage, cv::COLORMAP_JET);
    
    // Construct the output image
    cv::Mat outputImage(rgbImage.rows, rgbImage.cols, rgbImage.type());
    
    // Keep the original image if mask = 0
    for (size_t r = 0; r < rgbImage.rows; ++r) {
        for (size_t c = 0; c < rgbImage.cols; ++c) {
            if (mask.at<uchar>(r,c) < threshold) {
                // Keep the original image
                outputImage.at<cv::Vec3b>(r,c) = rgbImage.at<cv::Vec3b>(r,c);
                continue;
            }
            // Merge heatmap and original image if heatmap is greater than threshold
            for (size_t chan = 0; chan < rgbImage.channels(); ++chan) {
                outputImage.at<cv::Vec3b>(r,c)[chan] =
                    cv::saturate_cast<uchar>((1.0 - alpha) * rgbImage.at<cv::Vec3b>(r,c)[chan] +
                                             alpha * heatmapImage.at<cv::Vec3b>(r,c)[chan]);
            }
        }
    }
    return outputImage;
}

// Convert to gray pixel if whose value below threshold, otherwise keep the original RGB image
cv::Mat MergeBySwithingRGBAndGray(const cv::Mat& rgbImage, const cv::Mat& rawPropImage, int threshold) {
    // Assign pixels whose value below threshold to 0.
    cv::Mat mask; // result in 1 channel image
    cv::threshold(rawPropImage, mask, threshold, 255, cv::THRESH_TOZERO);
    
    // Convert the rgb image to gray
    cv::Mat graySingleChannel, gray;
    cv::cvtColor(rgbImage, graySingleChannel, cv::COLOR_BGR2GRAY);
    std::vector<cv::Mat> mergedImage = {graySingleChannel, graySingleChannel, graySingleChannel};
    cv::merge(mergedImage, gray);
    
    // Construct the output image
    cv::Mat outputImage(rgbImage.rows, rgbImage.cols, rgbImage.type());
    
    // Output the gray pixels if their value below threshold, original RGB whose value higher than threshold
    for (size_t r = 0; r < rgbImage.rows; ++r) {
        for (size_t c = 0; c < rgbImage.cols; ++c) {
            outputImage.at<cv::Vec3b>(r,c) = (mask.at<uchar>(r,c) < threshold) ?
                                gray.at<cv::Vec3b>(r,c) : rgbImage.at<cv::Vec3b>(r,c);
        }
    }
    return outputImage;
}
