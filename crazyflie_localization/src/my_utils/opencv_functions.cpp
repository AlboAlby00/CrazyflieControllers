#include "crazyflie_localization/my_utils/opencv_functions.h"

double my_utils::calculate_distance(const cv::Point2f &p1, const cv::Point2f &p2)
{
    double dx = p1.x - p2.x, dy = p1.y - p2.y;
    return sqrt(dx * dx + dy * dy);
}

void my_utils::save_image(const cv::Mat& image)
{
    static int count = 0;
    std::string path = "/home/alboalby00/ros2_galactic_ws/src/deep_learning_in_robotics/crazyflie_localization/images/";
    std::string name = path + std::to_string(count) + ".png";
    std::cout << "Saving image: " << name << std::endl;
    cv::imwrite(name, image);
    count++;
}