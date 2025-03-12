#include <opencv2/opencv.hpp>


class ADS {
public:
    ADS();
    void processDetection( void );
private:
    cv::VideoCapture cap;
};