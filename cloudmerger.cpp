#include "cloudmerger.h"

// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
  MyPointRepresentation ()
  {
    // Define the number of dimensions
    nr_dimensions_ = 4;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray (const PointNormalT &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};

CloudMerger::CloudMerger()
{
    mergeCloud = PointCloudT::Ptr(new PointCloudT);

    icp.setTransformationEpsilon(1e-6);
    icp.setMaxCorrespondenceDistance(0.1);
    icp.setMaximumIterations(20);
    //icp.setRANSACOutlierRejectionThreshold(0.1);
    sor.setLeafSize (0.05f, 0.05f, 0.05f);
    transformMatrix = Eigen::Affine3f::Identity();
}


PointCloudT::Ptr CloudMerger::getPairAlign(PointCloudT::Ptr cloud_src, PointCloudT::Ptr cloud_target, PointCloudT::Ptr output)
{
    PointCloudT final;
    PointCloudT::Ptr src (new PointCloudT);
    PointCloudT::Ptr tgt (new PointCloudT);
    pcl::VoxelGrid<PointT> grid;

      grid.setLeafSize (0.025, 0.025, 0.025);
      grid.setInputCloud (cloud_src);
      grid.filter (*src);

      grid.setInputCloud (cloud_target);
      grid.filter (*tgt);



    icp.setInputCloud(src);
    icp.setInputTarget(tgt);
    icp.align(*src);

    if(icp.hasConverged())
    {
        transformPointCloud(*cloud_src,final,icp.getFinalTransformation());

        final += *cloud_target;

        grid.setLeafSize (0.001, 0.001, 0.001);
        grid.setInputCloud (final.makeShared());
        grid.filter (*output);
    }


    //output->swap(final);

    return output;
}

PointCloudT::Ptr CloudMerger::getMergeCloud2(PointCloudT::ConstPtr cloud)
{
    if(cloud->size() == 0)
    {
        return mergeCloud;
    }

    if(mergeCloud->size() > 0)
    {
        PointCloudT targetCloud;
        PointCloudT srcCloud;
        Eigen::Matrix4f trans;
        copyPointCloud(*mergeCloud,targetCloud);
        transformPointCloud(*cloud,srcCloud,transformMatrix);
        //mergeCloud->swap(srcCloud);
        this->getPairAlign(srcCloud.makeShared(),targetCloud.makeShared(),mergeCloud);
    }else
    {
        copyPointCloud<PointT,PointT>(*cloud,*mergeCloud);
    }
    return mergeCloud;
}


void CloudMerger::pairAlign (PointCloudT::Ptr cloud_src, PointCloudT::Ptr cloud_tgt, PointCloudT::Ptr output, Eigen::Matrix4f &final_transform, bool downsample)
{
  //
  // Downsample for consistency and speed
  // \note enable this for large datasets
  PointCloudT::Ptr src (new PointCloudT);
  PointCloudT::Ptr tgt (new PointCloudT);
  pcl::VoxelGrid<PointT> grid;
  if (downsample)
  {
    grid.setLeafSize (0.05, 0.05, 0.05);
    grid.setInputCloud (cloud_src);
    grid.filter (*src);

    grid.setInputCloud (cloud_tgt);
    grid.filter (*tgt);
  }
  else
  {
    src = cloud_src;
    tgt = cloud_tgt;
  }


  // Compute surface normals and curvature
  PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
  PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

  pcl::NormalEstimationOMP<PointT, PointNormalT> norm_est;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
  norm_est.setSearchMethod (tree);
  norm_est.setKSearch (30);

  norm_est.setInputCloud (src);
  norm_est.compute (*points_with_normals_src);
  pcl::copyPointCloud (*src, *points_with_normals_src);

  norm_est.setInputCloud (tgt);
  norm_est.compute (*points_with_normals_tgt);
  pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

  //
  // Instantiate our custom point representation (defined above) ...
  MyPointRepresentation point_representation;
  // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues (alpha);

  //
  // Align
  pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
  reg.setTransformationEpsilon (1e-6);
  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  // Note: adjust this based on the size of your datasets
  reg.setMaxCorrespondenceDistance (0.1);
  // Set the point representation
  reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

  reg.setInputSource (points_with_normals_src);
  reg.setInputTarget (points_with_normals_tgt);



  //
  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
  reg.setMaximumIterations (2);
  for (int i = 0; i < 30; ++i)
  {
    PCL_INFO ("Iteration Nr. %d.\n", i);

    // save cloud for visualization purpose
    points_with_normals_src = reg_result;

    // Estimate
    reg.setInputSource (points_with_normals_src);
    reg.align (*reg_result);

        //accumulate transformation between each Iteration
    Ti = reg.getFinalTransformation () * Ti;

        //if the difference between this transformation and the previous one
        //is smaller than the threshold, refine the process by reducing
        //the maximal correspondence distance
    if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
      reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);

    prev = reg.getLastIncrementalTransformation ();

    // visualize current state
    //showCloudsRight(points_with_normals_tgt, points_with_normals_src);
  }

    //
  // Get the transformation from target to source
  targetToSource = Ti.inverse();

  //
  // Transform target back in source frame
  pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);

  //add the source to the transformed target
  *output += *cloud_src;

  final_transform = targetToSource;
 }

void CloudMerger::setTransformMatrix(float x, float y, float z, float rX, float rY, float rZ)
{
    Eigen::Affine3f transformMatrix = Eigen::Affine3f::Identity();;
    transformMatrix.translation() << x, y, z;
    transformMatrix.rotate (Eigen::AngleAxisf (rX, Eigen::Vector3f::UnitX()));
    transformMatrix.rotate (Eigen::AngleAxisf (rY, Eigen::Vector3f::UnitY()));
    transformMatrix.rotate (Eigen::AngleAxisf (rZ, Eigen::Vector3f::UnitZ()));

    this->transformMatrix = transformMatrix;
}


