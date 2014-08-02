#include "cloudmerger.h"
#include "mapperconfig.h"
#include <omp.h>
#include <pcl/filters/median_filter.h>

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
    lastCloud = PointCloudT::Ptr(new PointCloudT);

    CloudMergerConfig cmc = MapperConfig::getInstance().getCloudMergerConfig();

    icp.setTransformationEpsilon(cmc.transformationEpsilon);
    icp.setMaxCorrespondenceDistance(cmc.maxCorrespondenceDistance);
    icp.setMaximumIterations(cmc.maximumIterations);
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

    grid.setLeafSize (0.03, 0.03, 0.03);
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
        copyPointCloud(final,*cloud_target);

        //final += *cloud_target;
        final += *output;

        grid.setLeafSize (0.005, 0.005, 0.005);
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

    MedianFilter<PointT> mf;
    PointCloudT::Ptr filterCloud = PointCloudT::Ptr(new PointCloudT);
    mf.setWindowSize(10);
    mf.setInputCloud(cloud);
    mf.applyFilter(*filterCloud);

    //colorNormalizer2(filterCloud);

    if(mergeCloud->size() > 0)
    {
        PointCloudT targetCloud;
        PointCloudT srcCloud;

        //copyPointCloud(*mergeCloud,targetCloud);
        transformPointCloud(*filterCloud,srcCloud,transformMatrix);
        //mergeCloud->swap(srcCloud);
        this->getPairAlign(srcCloud.makeShared(),lastCloud,mergeCloud);
    }else
    {
        copyPointCloud<PointT,PointT>(*filterCloud,*mergeCloud);

        copyPointCloud<PointT,PointT>(*mergeCloud,*lastCloud);

    }
    return mergeCloud;
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

void CloudMerger::colorNormalizer(PointCloudT::Ptr cloud)
{
   // #pragma omp parallel for
    for(int i = 0; i<cloud->size(); i++)
    {
        PointT& point = cloud->points[i];
        float r,g,b,h,s,v;

        RGB_to_HSV(point.r,point.g,point.b,&h,&s,&v);

        HSV_to_RGB(h,s,0.75f,&r,&g,&b);

        point.r = int(r);
        point.g = int(g);
        point.b = int(b);
    }
}

void CloudMerger::colorNormalizer2(PointCloudT::Ptr cloud)
{
   // #pragma omp parallel for
    for(int i = 0; i<cloud->size(); i++)
    {
        PointT& point = cloud->points[i];
        float r,g,b,y,alpha;

        r = point.r;
        g = point.g;
        b = point.b;

        y = 0.2126*r + 0.7152*g + 0.0722*b;
        alpha = 100.0f/y;

        r *= alpha;
        g *= alpha;
        b *= alpha;

        if(r <= 255.0f)
        {
            point.r = int(r);
        }else
        {
            point.r = 255;
        }

        if(g <= 255.0f)
        {
            point.g = int(g);
        }else
        {
            point.g = 255;
        }

        if(b <= 255.0f)
        {
            point.b = int(b);
        }else
        {
            point.b = 255;
        }


    }
}


void CloudMerger::HSV_to_RGB(float h, float s, float v, float* r, float* g, float* b)
{
    float c = s * v;
    float x = c * (1.0f - fabs(fmod((h / 60.0f),2.0f) - 1.0f));
    float m = v - c;

    if( h < 60.0f)
    {
        *r = c;
        *g = x;
        *b = 0;
    }else if(h < 120.0f)
    {
        *r = x;
        *g = c;
        *b = 0;
    }else if(h < 180.0f)
    {
        *r = 0;
        *g = c;
        *b = x;
    }else if(h < 240.0f)
    {
        *r = 0;
        *g = x;
        *b = c;
    }else if(h < 300.0f)
    {
        *r = x;
        *g = 0;
        *b = c;
    }else
    {
        *r = c;
        *g = 0;
        *b = x;
    }

    *r += m;
    *g += m;
    *b += m;

    *r *= 255.0f;
    *g *= 255.0f;
    *b *= 255.0f;
}

void CloudMerger::RGB_to_HSV(float r, float g, float b, float* h, float* s, float* v)
{
    float r_prime = r/255.0f;
    float g_prime = g/255.0f;
    float b_prime = b/255.0f;

    float max = std::max(std::max(r_prime,g_prime),b_prime);
    float min = std::min(std::min(r_prime,g_prime),b_prime);

    float delta = max - min;
    if(delta == 0)
    {
       *h = 0;
       *s = 0;
       *v = max;
       return;
    }

    if(max == r_prime)
    {
        *h = 60.0f * fmod((g-b/delta),6.0f);
    }else if(max == g_prime)
    {
        *h = 60.0f*((b - r)/delta + 2.0f);
    }else
    {
        *h = 60*((r - g)/delta + 4.0f);
    }

    *s = delta/max;
    *v = max;
}


void CloudMerger::saveMap(std::string filePath)
{
    io::savePCDFile(filePath,*mergeCloud);
    io::savePCDFile("last_cloud",*lastCloud);
}

PointCloudT::Ptr CloudMerger::loadMap(std::string filePath)
{
    io::loadPCDFile(filePath,*mergeCloud);
    io::loadPCDFile("last_cloud",*lastCloud);
    return mergeCloud;
}



