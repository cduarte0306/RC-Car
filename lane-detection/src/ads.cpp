#include "ads.hpp"

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <opencv2/dnn.hpp>


ADS::ADS() {
    /* Initialize the video capture */
    cap.open("4608285-uhd_3840_2160_24fps.mp4"); // Change this to the path of your video file
    if ( !cap.isOpened() ) {
        std::cerr << "Error opening video stream or file" << std::endl;
        throw std::runtime_error("Error opening video stream or file");
    }
}


/**
 * @brief Main lane detection routine
 * 
 */
void ADS::processDetection( void ) {
    while (true) {
        /* Load the next frame in the video */
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) {
            break;
        }

        /* Show the frame */
        cv::imshow("Frame", frame);

        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

    cap.release();
    cv::destroyAllWindows();

}