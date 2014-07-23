#ifndef CLOUDMERGER_H
#define CLOUDMERGER_H

#include <pcl/search/kdtree.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/gicp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

#include "ICamera.h"

using namespace pcl;



class CloudMerger
{
public:
    CloudMerger();

    PointCloudT::Ptr getPairAlign(PointCloudT::Ptr cloud_src, PointCloudT::Ptr cloud_target, PointCloudT::Ptr output);
    PointCloudT::Ptr getMergeCloud2(PointCloudT::ConstPtr cloud);
    void setTransformMatrix(float x, float y, float z, float rX, float rY, float rZ);

private:
    PointCloudT::Ptr mergeCloud;
    GeneralizedIterativeClosestPoint<PointT,PointT> icp;
    pcl::VoxelGrid<PointT> sor;
    Eigen::Affine3f transformMatrix;
    void pairAlign(PointCloudT::Ptr cloud_src, PointCloudT::Ptr cloud_tgt, PointCloudT::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false);
};

#endif // CLOUDMERGER_H
