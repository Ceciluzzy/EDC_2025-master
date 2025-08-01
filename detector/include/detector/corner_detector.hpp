#ifndef CORNER_DETECTOR_HPP_
#define CORNER_DETECTOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include "opencv2/imgproc/imgproc.hpp"  
#include "opencv2/highgui/highgui.hpp"  
#include <opencv2/opencv.hpp>
#include <iostream>  
#include <cmath>

class CornerDetector
{
public:
    std::vector<cv::Point2f> Corner(const cv::Mat& img,
                                    cv::Mat& black_mask_out);
    
private:
    void sortCorners(std::vector<cv::Point2f>& corners);
    cv::Point2f computeIntersection(const std::vector<cv::Point2f>& corners);
};

#endif