#include "crazyflie_localization/utils/ORB_tracker.h"

my_vo::ORB_tracker::ORB_tracker() :
    detector(cv::ORB::create()),
    matcher(cv::DescriptorMatcher::create("BruteForce-Hamming"))
{

}

void my_vo::ORB_tracker::track_features(
    const cv::Mat& image_1, const cv::Mat& image_2,
    std::vector<cv::KeyPoint>& keypoints_1, std::vector<cv::KeyPoint>& keypoints_2,
    std::vector<cv::DMatch>& matches)
{
    cv::Mat descriptors_1, descriptors_2;

    detector->detectAndCompute(image_1, cv::noArray(), keypoints_1, descriptors_1);
    detector->detectAndCompute(image_2, cv::noArray(), keypoints_2, descriptors_2);   

    matcher->match(descriptors_1, descriptors_2, matches);

    //−− sort and remove the outliers

    auto min_max = minmax_element(matches.begin(), matches.end(),
        [](const cv::DMatch &m1, const cv::DMatch &m2) { return m1.distance < m2.distance; });
    double min_dist = min_max.first->distance;

    std::vector<cv::DMatch> good_matches;
    for (int i = 0; i < descriptors_1.rows; i++) {
        if (matches[i].distance <= std::max(2 * min_dist, 30.0)) {
            good_matches.push_back(matches[i]);
        }
    }


    matches = good_matches;

}