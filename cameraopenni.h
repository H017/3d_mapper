#ifndef CAMERAOPENNI_H
#define CAMERAOPENNI_H

#include "ICamera.h"
#include <pcl/io/openni_grabber.h>
#include <pcl/io/io.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <pcl/filters/fast_bilateral_omp.h>
#include <deque>

#define MAX_CLOUDS_SIZE 5
using namespace pcl;

class CameraOpenni : public ICamera
{
public:
    CameraOpenni();
    ~CameraOpenni();
    virtual PointCloudT::Ptr getCloud();

private:
    Grabber* interface;
    PointCloudT::ConstPtr currentCloud;
    void cloudCallback(const PointCloud<PointXYZRGBA>::ConstPtr&);
    boost::mutex m;
    std::deque<PointCloudT::Ptr> clouds;
    FastBilateralFilterOMP<PointT> filter;


};

#endif // CAMERAOPENNI_H
