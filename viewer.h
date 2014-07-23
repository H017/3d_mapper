#ifndef VIEWER_H
#define VIEWER_H

#include <QVTKWidget.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindow.h>
#include <boost/thread.hpp>

#include "ICamera.h"
#include "cameraopenni.h"
#include "cloudmerger.h"
#include <QObject>

using namespace pcl;
using namespace std;

class Viewer:public QObject
{

    Q_OBJECT

public:
    Viewer();
    ~Viewer();
    QVTKWidget& getWidget();
    void viewer_loop();
    CloudMerger* getCloudMerger();

public slots:
    void snapshot();
private:
    visualization::PCLVisualizer::Ptr visualizer;
    QVTKWidget viewer;
    volatile bool viewer_thread_active;
    boost::thread* viewer_thread;
    ICamera* cam;
    PointCloudT::Ptr currentMergeCloud;
    boost::mutex m;
    CloudMerger cm;


};

#endif // VIEWER_H
