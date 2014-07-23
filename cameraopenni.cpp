 #include "cameraopenni.h"


CameraOpenni::CameraOpenni()
{
    interface = new OpenNIGrabber();
    currentCloud = PointCloudT::Ptr(new PointCloudT);
    boost::function<void (const PointCloud<PointXYZRGBA>::ConstPtr&)> f =
    boost::bind(&CameraOpenni::cloudCallback, this, _1);

    interface->registerCallback(f);

    interface->start();
}

CameraOpenni::~CameraOpenni()
{
    interface->stop();
}

PointCloudT::Ptr CameraOpenni::getCloud()
{
   return clouds[0];
}

void CameraOpenni::cloudCallback(const PointCloud<PointXYZRGBA>::ConstPtr & cloud)
{
    if(cloud == NULL || cloud.get() == NULL || cloud->size() == 0) return;

    PointCloudT::Ptr current_cloud(new PointCloudT);
    PointCloudT::Ptr filter_cloud(new PointCloudT);
    copyPointCloud(*cloud,*current_cloud);

    filter.setInputCloud(current_cloud);

    filter.applyFilter(*filter_cloud);
    clouds.push_front(filter_cloud);
    if(clouds.size() > MAX_CLOUDS_SIZE)
    {
        clouds.pop_back();
    }
}
