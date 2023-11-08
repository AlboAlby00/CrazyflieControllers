#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>


namespace my_geom{

    cv::Point2f pixel2CamNormPlane(const cv::Point2f &p, const cv::Mat &K);
    cv::Point3f pixel2cam(const cv::Point2f &p, const cv::Mat &K, double depth = 1);
    cv::Point2f cam2pixel(const cv::Point3f &p, const cv::Mat &K);
    cv::Point2f cam2pixel(const cv::Mat &p, const cv::Mat &K);
    cv::Mat world2camera(const cv::Point3f &p, const cv::Mat &T_world_to_cam);
    
}