#include "crazyflie_localization/my_visual_odometry/matching_and_tracking.h"

void my_vo::estimate_pose(
    const std::vector<cv::KeyPoint>& keypoints_1, const std::vector<cv::KeyPoint>& keypoints_2, const cv::Mat& K,
                const std::vector<cv::DMatch>& matches, cv::Mat &R, cv::Mat &t)
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
    double focal_length =  K.at<double>(0,0); // camera focal length
    cv::Mat essential_matrix, mask;
    essential_matrix = cv::findEssentialMat(
        points1, points2, focal_length,principal_point, cv::RANSAC, 0.999, 1.0, mask);
    
    cv::recoverPose(
        essential_matrix, points1, points2, R, t, focal_length, principal_point, mask);
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

    return mean_distance;
}

void my_vo::get_descriptor_of_points_3d(const cv::Mat& descriptors, const std::vector<cv::DMatch>& matches, 
                cv::Mat& points_3d_descriptors)
{
    std::cout << "get_descriptor_of_points_3d" << std::endl;
    points_3d_descriptors = cv::Mat(matches.size(), descriptors.cols, descriptors.type());
    for (int i = 0; i < matches.size(); i++) {
        int query_idx = matches[i].trainIdx;
        descriptors.row(query_idx).copyTo(points_3d_descriptors.row(i));
    }

    //std::cout << "points_3d_descriptors " << points_3d_descriptors.rowRange(0,15) << std::endl;
    //std::cout << "frame descriptors " << descriptors.rowRange(0,15) << std::endl;
}

void my_vo::find_2d_3d_correspondences(const my_ds::Map::Ptr& map, const Frame::Ptr frame, 
        std::vector<cv::Point2f>& points_2d, std::vector<cv::Point3f>& points_3d, std::vector<cv::DMatch>& matches_2d_3d)
{
    std::cout << "find_2d_3d_correspondences" << std::endl;
    std::vector<my_ds::MapPoint::Ptr> candidate_map_points_in_map;
    std::vector<cv::Point2f> candidate_2d_points_in_image;
    std::cout << "map size is: " << map->_map_points.size() << std::endl;

    map->project_map_points_to_frame(
        frame, candidate_map_points_in_map, candidate_2d_points_in_image);
    //std::cout << "candidate_map_points_in_map.size(): " << candidate_map_points_in_map.size() << std::endl;

    std::vector<cv::KeyPoint> candidate_2d_keypoints_in_image;
    my_utils::convert_point2f_to_keypoint(candidate_2d_points_in_image, candidate_2d_keypoints_in_image);
    //std::cout << "candidate_2d_keypoints_in_image.size(): " << candidate_2d_keypoints_in_image.size() << std::endl;

    cv::Mat corresponding_map_points_descriptors = 
        my_utils::get_descriptors_of_map_points(candidate_map_points_in_map);

    std::cout << "candidate_map_points_in_map.size(): " << candidate_map_points_in_map.size() << std::endl;

    if(candidate_map_points_in_map.size() < 10)
    {
        std::cout << "there are not enough 2d 3d correspondences" << std::endl;
        return;
    }

    
    //std::cout << "corresponding_map_points_descriptors.size(): " << corresponding_map_points_descriptors.size() << std::endl;
    //std::cout << "corresponding_map_points_descriptors_2.size(): " << corresponding_map_points_descriptors_2.size() << std::endl;

    Frame::Ptr virtual_frame = std::make_shared<Frame>();
    virtual_frame->keypoints = candidate_2d_keypoints_in_image;
    virtual_frame->descriptors = corresponding_map_points_descriptors;
    virtual_frame->image = cv::Mat::zeros(frame->image.rows, frame->image.cols, CV_8UC3);

    track_features(virtual_frame, frame, matches_2d_3d);

    my_utils::show_matches(virtual_frame, frame, matches_2d_3d);

    for (const cv::DMatch& match : matches_2d_3d) 
    {
        int query_idx = match.queryIdx;
        int train_idx = match.trainIdx;
        my_ds::MapPoint::Ptr map_point = candidate_map_points_in_map[query_idx];
        cv::Point3f point_3d = map_point->_pos;
        cv::Point2f point_2d = frame->keypoints[train_idx].pt;
        points_2d.push_back(point_2d);
        points_3d.push_back(point_3d);
    }
  
}

bool my_vo::get_world_to_camera_T_by_Pnp(const std::vector<cv::Point3f>& points_3d, const std::vector<cv::Point2f>& points_2d, 
                const cv::Mat& K, cv::Mat& T)
{
    // not enough 2d 3d correspondences
    std::cout << "num 2d 3d correspondences: " << points_3d.size() << std::endl;
    if(points_3d.size() < 10) return false;

    cv::Mat rvec, tvec, R;
    bool useExtrinsicGuess = false;
    int iterationsCount = 100;
    float reprojectionError = 2.0;
    double confidence = 0.999;
    cv::Mat pnp_inliers_mask;
    cv::solvePnPRansac(points_3d, points_2d, K, cv::Mat(), rvec, tvec,
                        useExtrinsicGuess,iterationsCount, reprojectionError, confidence, pnp_inliers_mask);

    cv::Rodrigues(rvec, R);
    T = my_geom::convert_Rt_to_T(R,tvec);

    return true;
}