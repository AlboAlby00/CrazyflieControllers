#include "crazyflie_localization/my_utils/opencv_functions.h"
#include <sys/types.h>
#include <sys/stat.h>


double my_utils::calculate_distance(const cv::Point2f &p1, const cv::Point2f &p2)
{
    double dx = p1.x - p2.x, dy = p1.y - p2.y;
    return sqrt(dx * dx + dy * dy);
}

void my_utils::save_image(const cv::Mat& image)
{

    static int count = 0;
    std::string path = "/home/alboalby00/ros2_galactic_ws/src/deep_learning_in_robotics/crazyflie_localization/images/";

    struct stat info;
    bool folder_exists;
    if( stat( path.c_str(), &info ) != 0 ){
        printf( "cannot access %s\n", path.c_str() );
        return;
    }   
    else if( info.st_mode & S_IFDIR )  // S_ISDIR() doesn't exist on my windows 
        folder_exists = true;
    else
        folder_exists = false;

    
    if (!folder_exists)
    {
        std::string command = "mkdir " + path;
        system(command.c_str());
    }


    std::string name = path + std::to_string(count) + ".png";
    std::cout << "Saving image: " << name << std::endl;
    cv::imwrite(name, image);
    count++;
}

cv::Point3f my_utils::transform_point(const cv::Point3f &p3x1, const cv::Mat &T4x4)
{
    const cv::Mat &T = T4x4;
    double p[4] = {p3x1.x, p3x1.y, p3x1.z, 1};
    double res[3] = {0, 0, 0};
    for (int row = 0; row < 3; row++)
    {
        for (int j = 0; j < 4; j++)
            res[row] += T.at<double>(row, j) * p[j];
    }
    return cv::Point3f(res[0], res[1], res[2]);
}

void my_utils::convert_point2f_to_keypoint(const std::vector<cv::Point2f>& points, std::vector<cv::KeyPoint>& keypoints)
{
    for (const auto& point : points)
    {
        cv::KeyPoint keypoint;
        keypoint.pt = point;
        keypoints.push_back(keypoint);
    }
}

cv::Mat my_utils::get_descriptors_of_map_points(const std::vector<my_ds::MapPoint::Ptr>& map_points)
{
    std::vector<cv::Mat> descriptors;
    for (const auto& map_point : map_points)
    {
        descriptors.push_back(map_point->_descriptor);
        //std::cout << "map point descriptor " << map_point->_descriptor << std::endl;
    }
    cv::Mat descriptors_mat;
    cv::vconcat(descriptors, descriptors_mat);

    //std::cout << "descriptors_mat " << descriptors_mat.rowRange(0,10) << std::endl;

    return descriptors_mat;
}

void my_utils::show_matches(const my_vo::Frame::Ptr frame_1, const my_vo::Frame::Ptr frame_2, const std::vector<cv::DMatch>& matches)
{
    cv::Mat img_matches;
    cv::drawMatches(frame_1->image, frame_1->keypoints, frame_2->image, frame_2->keypoints, matches, img_matches);
    my_utils::save_image(img_matches);
}