#include "cloudmerger.h"
#include "mapperconfig.h"
#include <omp.h>
#include <pcl/filters/median_filter.h>
#include <pcl/segmentation/extract_clusters.h>
#include <limits>


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
    PointCloudT::Ptr filterCloud2 = PointCloudT::Ptr(new PointCloudT);
    PointCloudT::Ptr filterCloud = PointCloudT::Ptr(new PointCloudT);
    mf.setWindowSize(10);
    mf.setInputCloud(cloud);
    mf.applyFilter(*filterCloud2);

    EuclideanClusterExtraction<PointT> ece;
    ece.setSearchMethod(search::KdTree<PointT>::Ptr(new search::KdTree<PointT>));
    ece.setMaxClusterSize(INT_MAX);
    ece.setMinClusterSize(500);
    ece.setClusterTolerance(0.02);

    ece.setInputCloud(filterCloud2);
    std::vector<PointIndices> idx;

    ece.extract(idx);

    copyPointCloud(*filterCloud2,idx,*filterCloud);

    colorNormalizer2(filterCloud);

    if(mergeCloud->size() > 0)
    {
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
    HueBins hb;
    for(int i = 0; i<cloud->size(); i++)
    {
        addInHueBins(hb,&(cloud->points[i]));
    }


    for(int i = 0; i<hb.bins.size(); i++)
    {
        LuminosityBins* lb = hb.bins[i];
        int maxSize = 0;
        int maxIndex = 0;
        for(int j=0;j<lb->bins.size();j++)
        {
            int size = lb->bins[j]->size();
            if(maxSize < size)
            {
                maxSize = size;
                maxIndex = j;
            }
        }

        float yPrime = (float(maxIndex)+0.5f)*15.0f;

        for(int j=0;j<lb->bins.size();j++)
        {
            vectorPoint* v = lb->bins[j];
            for(int k =0;k<v->size();k++)
            {
                PointT* point = v->at(k);
                float r,g,b,y,alpha;

                r = point->r;
                g = point->g;
                b = point->b;

                y = 0.2126*r + 0.7152*g + 0.0722*b;
                alpha = yPrime/y;

                r *= alpha;
                g *= alpha;
                b *= alpha;

                if(r <= 255.0f)
                {
                    point->r = int(r);
                }else
                {
                    point->r = 255;
                }

                if(g <= 255.0f)
                {
                    point->g = int(g);
                }else
                {
                    point->g = 255;
                }

                if(b <= 255.0f)
                {
                    point->b = int(b);
                }else
                {
                    point->b = 255;
                }
            }
        }
    }
}

void CloudMerger::addInHueBins(HueBins &hb, PointT *pt)
{
    float r,g,b,h,s,v;
    r = float(pt->r);
    g = float(pt->g);
    b = float(pt->b);
    RGB_to_HSV(r,g,b,&h,&s,&v);
    float y = 0.2126*r + 0.7152*g + 0.0722*b;

    LuminosityBins* lb = hb.bins[0];

    float aa = 0;
    float delta = 360.0f/hb.bins.size();
    float bb = delta;
    for(int i=0;i<hb.bins.size();i++)
    {
        if(h>=aa && h<bb)
        {
            lb = hb.bins[i];
            break;
        }

        aa = bb;
        bb += delta;
    }

    if(y >= 0 && y < 15)
    {
        lb->bins[0]->push_back(pt);
    }else if(y >= 15 && y < 30)
    {
        lb->bins[1]->push_back(pt);
    }else if(y >= 30 && y < 45)
    {
        lb->bins[2]->push_back(pt);
    }else if(y >= 45 && y < 60)
    {
        lb->bins[3]->push_back(pt);
    }else if(y >= 60 && y < 75)
    {
        lb->bins[4]->push_back(pt);
    }else if(y >= 75 && y < 90)
    {
        lb->bins[5]->push_back(pt);
    }else if(y >= 95 && y < 105)
    {
        lb->bins[6]->push_back(pt);
    }else if(y >= 105 && y < 120)
    {
        lb->bins[7]->push_back(pt);
    }else if(y >= 120 && y < 135)
    {
        lb->bins[8]->push_back(pt);
    }else if(y >= 135 && y < 150)
    {
        lb->bins[9]->push_back(pt);
    }else if(y >= 150 && y < 165)
    {
        lb->bins[10]->push_back(pt);
    }else if(y >= 165 && y < 180)
    {
        lb->bins[11]->push_back(pt);
    }else if(y >= 180 && y < 195)
    {
        lb->bins[12]->push_back(pt);
    }else if(y >= 195 && y < 210)
    {
        lb->bins[13]->push_back(pt);
    }else if(y >= 210 && y < 225)
    {
        lb->bins[14]->push_back(pt);
    }else if(y >= 225 && y < 240)
    {
        lb->bins[15]->push_back(pt);
    }else
    {
        lb->bins[16]->push_back(pt);
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
        *h = 60.0f * fmod((g_prime-b_prime/delta),6.0f);
    }else if(max == g_prime)
    {
        *h = 60.0f*((b_prime - r_prime)/delta + 2.0f);
    }else
    {
        *h = 60*((r_prime - g_prime)/delta + 4.0f);
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



