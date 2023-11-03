#include "crazyflie_localization/my_visual_odometry/pose_estimator.h"

my_vo::PoseEstimator::PoseEstimator(cv::Mat K) :
    _K(K)
{
}

void my_vo::PoseEstimator::estimate( 
    std::vector<cv::KeyPoint> keypoints_1, std::vector<cv::KeyPoint> keypoints_2,
                std::vector<cv::DMatch> matches, cv::Mat &R, cv::Mat &t)
{
    //−− Convert the matching point to the form of vector<Point2f>
    std::vector<cv::Point2f> points1;
    std::vector<cv::Point2f> points2;
    for (int i = 0; i < (int) matches.size(); i++) 
    {
        points1.push_back(keypoints_1[matches[i].queryIdx].pt);
        points2.push_back(keypoints_2[matches[i].trainIdx].pt);
    }

    //−− Calculate essential matrix
    cv::Point2d principal_point(_K.at<double>(0,2),_K.at<double>(1,2)); // camera principal point
    double focal_length = _K.at<double>(0,0); // camera focal length, calibrated in TUM dataset
    cv::Mat essential_matrix;
    essential_matrix = cv::findEssentialMat(points1, points2, focal_length,principal_point);

    //−− Recover rotation and translation from the essential matrix.
    cv::recoverPose(essential_matrix, points1, points2, R, t, focal_length,
        principal_point);
}