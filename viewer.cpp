#include "viewer.h"
#include "mapperconfig.h"
#include "directoryreader.h"

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

    if(MapperConfig::getInstance().isLive())
    {
        cam = new CameraOpenni();
    }else
    {
        cam = new DirectoryReader();
    }

    viewer_thread = new boost::thread(&Viewer::viewer_loop,this);
}

Viewer::~Viewer()
{
    thread_running = false;
    while(!thread_stop)
    {
         boost::this_thread::sleep(boost::posix_time::millisec(10));
    }
    delete cam;
}

QVTKWidget& Viewer::getWidget()
{

    return viewer;
}
void Viewer::viewer_loop()
{
    thread_running = true;
    thread_stop = false;
    while(thread_running && !visualizer->wasStopped())
    {
        boost::this_thread::sleep(boost::posix_time::millisec(100));

        PointCloudT::ConstPtr cloud = cam->getCloud();
        if(cloud == NULL || cloud.get() == NULL || cloud->size() == 0) continue;

        PointCloudT filterCloud;
        std::vector<int> idx;
        removeNaNFromPointCloud(*cloud,filterCloud,idx);
        PointCloudT::Ptr cloudPtr(new PointCloudT);
        pcl::VoxelGrid<PointT> grid;

        grid.setLeafSize (0.01, 0.01, 0.01);
        grid.setInputCloud (filterCloud.makeShared());
        grid.filter (*cloudPtr);
        pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloudPtr);

        m.lock();
        visualizer->updatePointCloud<PointT>(cloudPtr,rgb,"cloud");
        if(!MapperConfig::getInstance().isLive())
        {
            currentMergeCloud = cm.getMergeCloud2(cloud);
            pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb2(currentMergeCloud);
            visualizer->updatePointCloud<PointT>(currentMergeCloud,rgb2,"cloud2");
        }
        viewer.update();
        m.unlock();
    }

    thread_stop = true;
}

void Viewer::snapshot()
{
    if(!MapperConfig::getInstance().isLive())
    {
        return;
    }

    currentMergeCloud = cm.getMergeCloud2(cam->getCloud());

    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb2(currentMergeCloud);
    m.lock();
    visualizer->updatePointCloud<PointT>(currentMergeCloud,rgb2,"cloud2");
    m.unlock();
}

void Viewer::saveMap()
{
    cm.saveMap();
}

void Viewer::loadMap()
{
    currentMergeCloud = PointCloudT::Ptr(new PointCloudT);

    pcl::VoxelGrid<PointT> grid;

    grid.setLeafSize (0.01, 0.01, 0.01);
    grid.setInputCloud (cm.loadMap());
    grid.filter (*currentMergeCloud);

    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb2(currentMergeCloud);
    m.lock();
    visualizer->updatePointCloud<PointT>(currentMergeCloud,rgb2,"cloud2");
    m.unlock();
}


CloudMerger* Viewer::getCloudMerger()
{
    return &cm;
}

