#include "cameraopenni.h"
#include "mapperconfig.h"


CameraOpenni::CameraOpenni()
{
    interface = new OpenNIGrabber();
    currentCloud = PointCloudT::Ptr(new PointCloudT);
    boost::function<void (const PointCloud<PointXYZRGBA>::ConstPtr&)> f =
    boost::bind(&CameraOpenni::cloudCallback, this, _1);
    CameraConfig cc = MapperConfig::getInstance().getCameraConfig();
    filter.setSigmaS(cc.sigmaS);
    filter.setSigmaR(cc.sigmaR);
    callback_loop_check = true;

    interface->registerCallback(f);

    interface->start();
}

CameraOpenni::~CameraOpenni()
{
    interface->stop();
}

PointCloudT::Ptr CameraOpenni::getCloud()
{
    boost::mutex::scoped_lock lock(m);
    if(clouds.size() > 0)
        return clouds[clouds.size()-1];
    else
        return PointCloudT::Ptr(new PointCloudT);
}

void CameraOpenni::cloudCallback(const PointCloud<PointXYZRGBA>::ConstPtr & cloud)
{
    if(callback_loop_check)
    {
        callback_loop_check = false;
        if(cloud == NULL || cloud.get() == NULL || cloud->size() == 0) return;

        PointCloudT::Ptr current_cloud(new PointCloudT);
        PointCloudT::Ptr filter_cloud(new PointCloudT);
        copyPointCloud(*cloud,*current_cloud);

        filter.setInputCloud(current_cloud);

        filter.applyFilter(*filter_cloud);
        m.lock();
        clouds.push_front(filter_cloud);
        if(clouds.size() > MAX_CLOUDS_SIZE)
        {
            clouds.pop_back();
        }
        m.unlock();
        callback_loop_check = true;
    }else
    {
        std::cout <<"callback too fast!"<<std::endl;
    }
}
