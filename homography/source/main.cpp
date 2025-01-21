//
//  main.cpp
//  homography
//
//  Created by Supannee Tanathong on 04/06/2024.
//

#include "homography.hpp"

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>
#include <vector>
#include <optional>
#include <iostream>

int main(int argc, char * argv[]) {
    // Example#1
    {
        // Program arguments
        size_t outputImageWidth = 1040;
        size_t outputImageHeight = 1500;
        std::string imagePath("./data/rainbow.jpg");
        std::string outputPath("./output");
        
        // Source points: image captured with tilt angle
        std::vector<Eigen::Vector2d> sourcePoints;
        sourcePoints.emplace_back(213,43); // top-left
        sourcePoints.emplace_back(715,182); // top-right
        sourcePoints.emplace_back(530,858); // bottom-right
        sourcePoints.emplace_back(56,758); // bottom-left
        sourcePoints.emplace_back(382,471);
        sourcePoints.emplace_back(286,360);
        sourcePoints.emplace_back(514,418);
        sourcePoints.emplace_back(349,377);
        sourcePoints.emplace_back(454,403);
        
        // Destination points: rectified image
        std::vector<Eigen::Vector2d> destinationPoints;
        destinationPoints.emplace_back(0,0); // top-left
        destinationPoints.emplace_back(1039,0); // top-right
        destinationPoints.emplace_back(1039,1499); // bottom-right
        destinationPoints.emplace_back(0,1499); // bottom-left
        destinationPoints.emplace_back(519,749);
        destinationPoints.emplace_back(269,569);
        destinationPoints.emplace_back(769,569);
        destinationPoints.emplace_back(409,569);
        destinationPoints.emplace_back(629,569);
        
        cv::Mat image = cv::imread(imagePath, cv::IMREAD_COLOR);
        if (image.empty()) {
            std::cout << "Error! Failed to read image " << imagePath << std::endl;
            return -1;
        }
        
        // DLT solution
        auto H = EstimateHomographyDLT(sourcePoints, destinationPoints, {}, {});
        if (H.has_value()) {
            std::cout << "Estimated homography matrix using DLT method:" << std::endl;
            std::cout << *H << std::endl;
        }
        
        cv::Mat outputImage = CreateHomographyImage(image, *H, outputImageWidth, outputImageHeight);
        cv::imwrite(outputPath + "/homographyImage-DLT.png", outputImage);
    }
    
    return 0;
}
