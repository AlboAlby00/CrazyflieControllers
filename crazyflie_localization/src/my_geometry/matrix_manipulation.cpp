#include "crazyflie_localization/my_geometry/matrix_manipulation.h"

void my_geom::get_T_matrix(const cv::Matx33d& R, const cv::Vec3d& t, cv::Matx44d& T )
{
    T.get_minor<3, 3>(0, 0) = R;
    T.get_minor<3, 1>(0, 3) = t;
}

void my_geom::get_T_matrix(
    const geometry_msgs::msg::PoseStamped::SharedPtr& pose, cv::Matx44d& T)
{
    
    // Extract the translation (x, y, z)
    cv::Vec3d t(pose->pose.position.x, pose->pose.position.y, pose->pose.position.z);
    cv::Vec4d quaternion(pose->pose.orientation.w, pose->pose.orientation.x, pose->pose.orientation.y, pose->pose.orientation.z);

}

void my_geom::get_3x4_T_matrix(const cv::Mat& R, const cv::Mat& t, cv::Mat& T)
{
    T = (cv::Mat_<float>(3, 4) <<
            R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
            R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
            R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0));
}

cv::Mat my_geom::get_3x4_T_matrix(const cv::Mat& T)
{
    cv::Mat T_3x4 = (cv::Mat_<double>(3, 4) <<
                    T.at<double>(0, 0), T.at<double>(0, 1), T.at<double>(0, 2), T.at<double>(0, 3),
                    T.at<double>(1, 0), T.at<double>(1, 1), T.at<double>(1, 2), T.at<double>(1, 3),
                    T.at<double>(2, 0), T.at<double>(2, 1), T.at<double>(2, 2), T.at<double>(2, 3));
    return T_3x4;
    
}

cv::Mat my_geom::get_3x4_identity()
{
    return (cv::Mat_<float>(3, 4) <<
            1, 0, 0, 0,
            0,  1, 0, 0,
            0, 0, 1, 0);
}

cv::Mat my_geom::convert_Rt_to_T(const cv::Mat &R, const cv::Mat &t)
{
    cv::Mat T = (cv::Mat_<double>(4, 4) << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
                 R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
                 R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0),
                 0, 0, 0, 1);
    return T;
}

void my_geom::convert_T_to_Rt(const cv::Mat &T, cv::Mat &R, cv::Mat &t)
{
    R = (cv::Mat_<double>(3, 3) << 
            T.at<double>(0, 0), T.at<double>(0, 1), T.at<double>(0, 2),
            T.at<double>(1, 0), T.at<double>(1, 1), T.at<double>(1, 2),
            T.at<double>(2, 0), T.at<double>(2, 1), T.at<double>(2, 2));
    t = (cv::Mat_<double>(3, 1) << T.at<double>(0, 3), T.at<double>(1, 3), T.at<double>(2, 3));
}


void my_geom::omogeneous_to_3d(const cv::Mat& points_4d, std::list<cv::Point3d>& points_3d)
{
    for ( int i=0; i<points_4d.cols; i++ )
    {
        cv::Mat x = points_4d.col(i);
        x /= x.at<float>(3,0); 
        cv::Point3d p (
            x.at<float>(0,0), 
            x.at<float>(1,0), 
            x.at<float>(2,0) 
        );
        points_3d.push_back( p );
    }
}

cv::Mat my_geom::convert_point3f_to_homogeneous(const cv::Point3f& point)
{
    cv::Mat omogeneous_point = (cv::Mat_<float>(4, 1) << point.x, point.y, point.z, 1);
    return omogeneous_point;
}


