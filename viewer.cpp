#include "viewer.h"

Viewer::Viewer()
{
    visualizer = visualization::PCLVisualizer::Ptr(new visualization::PCLVisualizer("Viewer"));
    int v1(0);
    visualizer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    visualizer->setBackgroundColor(0,0,0,v1);
    PointCloudT::Ptr cloud = PointCloudT::Ptr(new PointCloudT);
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
    visualizer->addPointCloud<PointT>(cloud,rgb,"cloud",v1);
    visualizer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE,3,"cloud");

    int v2(0);
    visualizer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    visualizer->setBackgroundColor(0.5,0.5,0.5,v2);
    PointCloudT::Ptr cloud2 = PointCloudT::Ptr(new PointCloudT);
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb2(cloud2);
    visualizer->addPointCloud<PointT>(cloud2,rgb2,"cloud2",v2);
    visualizer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE,3,"cloud2");
    visualizer->addCoordinateSystem(1.0);


    visualizer->initCameraParameters();

    currentMergeCloud = PointCloudT::Ptr(new PointCloudT);

    viewer.SetRenderWindow(visualizer->getRenderWindow());
//visualizer->setupInteractor(viewer.GetInteractor(),viewer.GetRenderWindow());

    viewer.update();
    viewer_thread_active = true;
    cam = new CameraOpenni();

    viewer_thread = new boost::thread(&Viewer::viewer_loop,this);
}

Viewer::~Viewer()
{
    delete cam;
}

QVTKWidget& Viewer::getWidget()
{
    return viewer;
}
void Viewer::viewer_loop()
{
    while(!visualizer->wasStopped())
    {
        boost::this_thread::sleep(boost::posix_time::millisec(100));

        PointCloudT::ConstPtr cloud = cam->getCloud();
        if(cloud == NULL || cloud.get() == NULL || cloud->size() == 0) continue;

        PointCloudT filterCloud;
        std::vector<int> idx;
        removeNaNFromPointCloud(*cloud,filterCloud,idx);
        PointCloudT::Ptr cloudPtr = filterCloud.makeShared();
        pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloudPtr);

        m.lock();
        visualizer->updatePointCloud<PointT>(cloudPtr,rgb,"cloud");
        viewer.update();
        m.unlock();
    }
}

void Viewer::snapshot()
{


    currentMergeCloud = cm.getMergeCloud2(cam->getCloud());

    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb2(currentMergeCloud);
    m.lock();
    visualizer->updatePointCloud<PointT>(currentMergeCloud,rgb2,"cloud2");
    m.unlock();
}

CloudMerger* Viewer::getCloudMerger()
{
    return &cm;
}

