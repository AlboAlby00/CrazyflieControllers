#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace my_ds
{
    
        class MapPoint
        {
        public:
            typedef std::shared_ptr<MapPoint> Ptr;
            
            MapPoint(cv::Point3f pos, cv::Mat descriptor);
            static MapPoint::Ptr createMapPoint(cv::Point3f pos, cv::Mat _descriptor);
            cv::Point3f _pos;
            cv::Mat _descriptor;   

            int _id;
            //std::vector<unsigned char> color_;
    
        private:
            static int _factory_id;

        };

}