#include "crazyflie_localization/my_visual_odometry/matching_and_tracking.h"

void my_vo::estimate_pose(
    std::vector<cv::KeyPoint> keypoints_1, std::vector<cv::KeyPoint> keypoints_2, cv::Mat K,
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
    cv::Point2d principal_point( K.at<double>(0,2), K.at<double>(1,2)); // camera principal point
    double focal_length =  K.at<double>(0,0); // camera focal length, calibrated in TUM dataset
    cv::Mat essential_matrix;
    essential_matrix = cv::findEssentialMat(points1, points2, focal_length,principal_point);

    //−− Recover rotation and translation from the essential matrix.
    cv::recoverPose(essential_matrix, points1, points2, R, t, focal_length,
        principal_point);
}

void my_vo::track_features(
        const Frame::Ptr frame_1, const Frame::Ptr frame_2,
                    std::vector<cv::DMatch>& matches)
{

    matcher->match(frame_1->descriptors, frame_2->descriptors, matches);

    //−− sort and remove the outliers
    auto min_max = minmax_element(matches.begin(), matches.end(),
        [](const cv::DMatch &m1, const cv::DMatch &m2) { return m1.distance < m2.distance; });
    double min_dist = min_max.first->distance;

    std::vector<cv::DMatch> good_matches;
    for (int i = 0; i < frame_1->descriptors.rows; i++) {
        if (matches[i].distance <= std::max(2 * min_dist, 30.0)) {
            good_matches.push_back(matches[i]);
        }
    }

    matches = good_matches;

}