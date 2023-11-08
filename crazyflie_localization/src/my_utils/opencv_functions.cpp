#include "crazyflie_localization/my_utils/opencv_functions.h"

double my_utils::calculate_distance(const cv::Point2f &p1, const cv::Point2f &p2)
{
    double dx = p1.x - p2.x, dy = p1.y - p2.y;
    return sqrt(dx * dx + dy * dy);
}