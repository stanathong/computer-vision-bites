//
//  main.cpp
//  superimpose-heatmap
//
//  Created by Supannee Tanathong on 22/01/2025.
//

#include <iostream>
#include "cv.hpp"

static void PrintHelp(const char** argv) {
    std::cout << "Superimpose the property image on top of the RGB image\n";
    std::cout << "Usage: " << argv[0] << " rgb-path property-path output-path [alpha] [discard-threshold] [merge-approach=0]\n";
    std::cout << "\trgb-path: Path to an rgb image or path to a folder containing RGB images\n";
    std::cout << "\tproperty-path: Path to an property image or path to a folder containing property images e.g. heatmap\n";
    std::cout << "\toutput-path: Path to the output directory, which will have the same name as the input image\n";
    std::cout << "\talpha: [optional] alpha value between 0.0 - 1.0 applied to the property image, default = 0.75\n";
    std::cout << "\tdiscard-threshold: [optional] when the property value is below this threshold, it is discarded (set to 0.0), default = 30\n";
    std::cout << "Note:\n";
    std::cout << "To render the whole folder, the RGB images and the property images must have the same name.\n";
    std::cout << std::endl;
}

int main(int argc, const char * argv[]) {
    if (argc < 4) {
        PrintHelp(argv);
        return -1;
    }
    double alpha = (argc > 4) ? std::atof(argv[4]) : 0.75;
    int threshold = (argc > 5) ? std::atoi(argv[5]) : 30;
    int approach = (argc > 6) ? std::atoi(argv[6]) : 0;
    return Process(argv[1], argv[2], argv[3], alpha, threshold, approach);
}
