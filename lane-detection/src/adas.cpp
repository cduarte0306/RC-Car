#include "adas.hpp"

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <opencv2/dnn.hpp>

#include "twinlitenet_dnn.hpp"


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
    TwinLiteNet twinlitenet("best.onnx");;

    while (true) {
        /* Load the next frame in the video */
        cv::Mat frame;
        cv::Mat da_out, ll_out;

        cap >> frame;
        if (frame.empty()) {
            break;
        }

        // Resize the image to 360x640 as the model expects this size
        // cv::resize(frame, frame, cv::Size(640, 360));
        // cv::Mat img_vis = frame.clone();

        // twinlitenet.Infer(frame, da_out, ll_out);
        
        // img_vis.setTo(cv::Scalar(255, 0, 127), da_out);
        // img_vis.setTo(cv::Scalar(0, 0, 255), ll_out);

        /* Show the frame */
        cv::imshow("Frame", frame);

        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

    cap.release();
    cv::destroyAllWindows();
}