// main_classimpl.cpp (example usage)
#include "fpv_liberator.h"

int main() {
    FPVLiberator fpv;
    
    if (!fpv.initialize()) {
        std::cerr << "Failed to initialize FPV Liberator" << std::endl;
        return 1;
    }

    fpv.run();

    cv::namedWindow("Debug Output", cv::WINDOW_NORMAL);
    while (true) {
        cv::Mat frame = fpv.getLatestFrame();
        // Decode the frame
        cv::Mat decoded_frame = cv::imdecode(frame, cv::IMREAD_COLOR);

        // Output the decoded frame
        cv::imshow("Debug Output", decoded_frame);
        cv::imshow("Debug Output", frame);
        if (cv::waitKey(1) == 27) break;  // Exit if ESC is pressed
    }

    fpv.stop();
    return 0;
}

