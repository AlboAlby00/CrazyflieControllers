#include "crazyflie_localization/my_datastructures/map_point.h"

int my_ds::MapPoint::_factory_id = 0;

my_ds::MapPoint::MapPoint(cv::Point3f pos, cv::Mat descriptor) : _pos(pos), _descriptor(descriptor) 
{}

my_ds::MapPoint::Ptr my_ds::MapPoint::createMapPoint(cv::Point3f pos, cv::Mat _descriptor)
{
    my_ds::MapPoint::Ptr map_point = std::make_shared<my_ds::MapPoint>(pos, _descriptor);
    map_point->_id = _factory_id;
    _factory_id++;
    return map_point;
}