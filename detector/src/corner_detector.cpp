#include "detector/corner_detector.hpp"

void CornerDetector::sortCorners(std::vector<cv::Point2f>& corners) {
    if (corners.size() != 4) return;

    // 计算中心点
    cv::Point2f center(0, 0);
    for (const auto& p : corners) {
        center += p;
    }
    center *= 1.0 / corners.size();

    // 按相对中心位置分类
    std::vector<cv::Point2f> top, bottom;
    for (const auto& p : corners) {
        if (p.y < center.y)
            top.push_back(p);
        else
            bottom.push_back(p);
    }

    // 左上（x较小）和右上（x较大）
    std::sort(top.begin(), top.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
        return a.x < b.x;
    });

    // 右下（x较大）和左下（x较小）
    std::sort(bottom.begin(), bottom.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
        return a.x > b.x;
    });

    std::vector<cv::Point2f> sorted;
    if (top.size() >= 2 && bottom.size() >= 2) {
        sorted.push_back(top[0]);    // 左上
        sorted.push_back(top[1]);    // 右上
        sorted.push_back(bottom[0]); // 右下
        sorted.push_back(bottom[1]); // 左下
    }
    corners = sorted;
}

cv::Point2f CornerDetector::computeIntersection(const std::vector<cv::Point2f>& corners) {
    if (corners.size() < 4) return cv::Point2f(-1, -1);
    
    // 定义四个顶点
    cv::Point2f tl = corners[0]; // 左上
    cv::Point2f tr = corners[1]; // 右上
    cv::Point2f br = corners[2]; // 右下
    cv::Point2f bl = corners[3]; // 左下

    // 计算两条对角线的参数：左上→右下，右上→左下
    cv::Vec4f line1(tl.x, tl.y, br.x, br.y); // 对角线 AC
    cv::Vec4f line2(tr.x, tr.y, bl.x, bl.y); // 对角线 BD

    // 计算交点
    float a1 = line1[3] - line1[1]; // dy1
    float b1 = line1[0] - line1[2]; // -dx1
    float c1 = a1 * line1[0] + b1 * line1[1];

    float a2 = line2[3] - line2[1]; // dy2
    float b2 = line2[0] - line2[2]; // -dx2
    float c2 = a2 * line2[0] + b2 * line2[1];

    float det = a1 * b2 - a2 * b1;
    if (fabs(det) < 1e-5) return cv::Point2f(-1, -1); // 避免平行线

    float x = (b2 * c1 - b1 * c2) / det;
    float y = (a1 * c2 - a2 * c1) / det;
    return cv::Point2f(x, y);
}

std::vector<cv::Point2f> CornerDetector::Corner(const cv::Mat &image,
                                                cv::Mat& bin_mask_out) 
{
    // 前4点为四边形角点，第5点为中心交点
    std::vector<cv::Point2f> corners_data(5, cv::Point2f(0, 0));
    
    cv::Mat img = image.clone();
    if (img.empty()) {
        std::cout << "无法读取图像！" << std::endl;
        return corners_data;
    }

    // 预处理图像 
    cv::Mat gray, blur_img, bin_img, edges;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blur_img, cv::Size(5, 5), 1);
    cv::threshold(blur_img, bin_img, 50, 255, cv::THRESH_BINARY);
    cv::Canny(bin_img, edges, 70, 110);
    
    bin_mask_out = bin_img.clone();
    
    // 使用边缘图像查找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    if (contours.empty()) {
        std::cout << "未检测到任何轮廓！" << std::endl;
        return corners_data;
    }
    
    // 过滤轮廓
    std::vector<std::vector<cv::Point>> filtered_contours;
    for (const auto& cnt : contours) {
        double area = cv::contourArea(cnt);
        if (area > 2000) {
            filtered_contours.push_back(cnt);
        }
    }
    
    if (filtered_contours.empty()) {
        std::cout << "未检测到足够大的轮廓！" << std::endl;
        return corners_data;
    }
    
    // 检测四边形
    std::vector<std::vector<cv::Point>> rectangles;
    for (const auto& cnt : filtered_contours) {
        double peri = cv::arcLength(cnt, true);
        std::vector<cv::Point> approx;
        cv::approxPolyDP(cnt, approx, 0.02 * peri, true);
        
        if (approx.size() == 4) {
            rectangles.push_back(approx);
        }
    }
    
    if (rectangles.empty()) {
        std::cout << "未检测到四边形！" << std::endl;
        return corners_data;
    }
    
    // 找到面积最大的四边形
    auto rect = *std::max_element(rectangles.begin(), rectangles.end(),
        [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
            return cv::contourArea(a) < cv::contourArea(b);
        });
    
    // 将整数点转换为浮点
    std::vector<cv::Point2f> paper_corners;
    for (const auto& p : rect) {
        paper_corners.push_back(cv::Point2f(p.x, p.y));
    }
    
    // 对四边形角点进行排序
    sortCorners(paper_corners);
    
    // 存储四边形角点
    for (int i = 0; i < 4; i++) {
        corners_data[i] = paper_corners[i];
    }
    
    // 计算对角线交点 
    cv::Point2f intersection = computeIntersection(paper_corners);
    corners_data[4] = intersection;
    
    return corners_data;
}