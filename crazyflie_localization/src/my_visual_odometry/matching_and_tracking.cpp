#include "crazyflie_localization/my_visual_odometry/matching_and_tracking.h"

void my_vo::estimate_pose(
    const std::vector<cv::KeyPoint> keypoints_1, const std::vector<cv::KeyPoint> keypoints_2, const cv::Mat& K,
                const std::vector<cv::DMatch>& matches, cv::Mat &R, cv::Mat &t)
{
    std::cout << "f0" << std::endl;
    //−− Convert the matching point to the form of vector<Point2f>
    std::vector<cv::Point2f> points1;
    std::vector<cv::Point2f> points2;
    for (int i = 0; i < (int) matches.size(); i++) 
    {
        points1.push_back(keypoints_1[matches[i].queryIdx].pt);
        points2.push_back(keypoints_2[matches[i].trainIdx].pt);
    }
    std::cout << "f1" << std::endl;

    //−− Calculate essential matrix
    cv::Point2d principal_point( K.at<double>(0,2), K.at<double>(1,2)); // camera principal point
    double focal_length =  K.at<double>(0,0); // camera focal length
    cv::Mat essential_matrix;
    essential_matrix = cv::findEssentialMat(points1, points2, focal_length,principal_point);

    std::cout << essential_matrix << std::endl;
    //−− Recover rotation and translation from the essential matrix.
    cv::recoverPose(essential_matrix, points1, points2, R, t, focal_length,
        principal_point);

    std::cout << "f3" << std::endl;
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

void my_vo::convert_keypoints_to_point2f(
    const std::vector<cv::KeyPoint>& keypoints_1 , const std::vector<cv::KeyPoint>& keypoints_2 , const cv::Mat K,
            std::vector<cv::DMatch>& matches, std::vector<cv::Point2f>& points_1, std::vector<cv::Point2f>& points_2)
{
    for ( cv::DMatch m: matches )
    {
        points_1.push_back( my_geom::pixel2CamNormPlane( keypoints_1[m.queryIdx].pt, K) );
        points_2.push_back( my_geom::pixel2CamNormPlane( keypoints_2[m.trainIdx].pt, K) );
    }
}

double my_vo::compute_keypoints_mean_distance(
                const std::vector<cv::KeyPoint> keypoints_1, const std::vector<cv::KeyPoint> keypoints_2, 
                    const std::vector<cv::DMatch>& matches)
{
    double total_distance = 0.0;
    for (const cv::DMatch& match : matches) {
        cv::Point2f pt1 = keypoints_1[match.queryIdx].pt;
        cv::Point2f pt2 = keypoints_2[match.trainIdx].pt;
        
        double distance = my_utils::calculate_distance(pt1, pt2); // Euclidean distance between keypoints
        total_distance += distance;
    }
    double mean_distance = total_distance / static_cast<double>(matches.size());
    std::cout << "mean_distance: " << mean_distance << std::endl;

    return mean_distance;
}