#ifndef CLOUDMERGER_H
#define CLOUDMERGER_H

#include <pcl/search/kdtree.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/gicp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include "ICamera.h"
using namespace std;
using namespace pcl;

typedef std::vector<PointT*> vectorPoint;

class LuminosityBins
{
public:
    vector<vectorPoint*> bins;
    LuminosityBins()
    {
        for(int i=0;i<17;i++)
        {
            bins.push_back(new vectorPoint);
        }
    }

    ~LuminosityBins()
    {
        for(int i=0;i<17;i++)
        {
            //delete bins[0];
        }

        bins.clear();
    }


};

class HueBins
{
public:
    std::vector<LuminosityBins*> bins;
    HueBins()
    {
        for(int i =0;i<20;i++)
        {
            bins.push_back(new LuminosityBins);
        }
    }

    ~HueBins()
    {
        for(int i =0;i<10;i++)
        {
            //delete bins[i];
        }
        bins.clear();
    }



};



class CloudMerger
{
public:
    CloudMerger();

    PointCloudT::Ptr getPairAlign(PointCloudT::Ptr cloud_src, PointCloudT::Ptr cloud_target, PointCloudT::Ptr output);
    PointCloudT::Ptr getMergeCloud2(PointCloudT::ConstPtr cloud);
    void setTransformMatrix(float x, float y, float z, float rX, float rY, float rZ);
    void saveMap(std::string filePath="map_3d.pcd");
    PointCloudT::Ptr loadMap(std::string filePath="map_3d.pcd");
    void HSV_to_RGB(float h, float s, float v, float* r, float* g, float* b);
    void RGB_to_HSV(float r, float g, float b, float* h, float* s, float* v);

private:
    PointCloudT::Ptr mergeCloud;
    PointCloudT::Ptr lastCloud;
    GeneralizedIterativeClosestPoint<PointT,PointT> icp;
    pcl::VoxelGrid<PointT> sor;
    Eigen::Affine3f transformMatrix;


    void addInHueBins(HueBins& hb, PointT *pt);
    void colorNormalizer(PointCloudT::Ptr cloud);
    void colorNormalizer2(PointCloudT::Ptr cloud);

};

#endif // CLOUDMERGER_H
