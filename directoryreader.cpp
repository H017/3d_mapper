#include "directoryreader.h"
#include <dirent.h>
#include <pcl/io/pcd_io.h>

DirectoryReader::DirectoryReader(std::string directoryPath)
{
    this->directoryPath = directoryPath;
    count = -1;
    loadFiles();
}

PointCloudT::Ptr DirectoryReader::getCloud()
{
    count++;
    count %= files.size();
    if((count+1) > clouds.size())
    {
        PointCloudT::Ptr cloud = PointCloudT::Ptr(new PointCloudT);
        io::loadPCDFile(files[count],*cloud);
        clouds.push_back(cloud);
        return clouds[count];
    }else
    {
        return PointCloudT::Ptr(new PointCloudT);
    }


}

void DirectoryReader::loadFiles()
{
    DIR* dir;
    struct dirent *file;
    dir = opendir(directoryPath.c_str());
    if(dir != NULL)
    {
        while((file = readdir(dir)) != NULL)
        {
            std::string filepath = file->d_name;
            if(filepath.find(".pcd")!= std::string::npos)
            {
                files.push_back(directoryPath+filepath);
                std::sort(files.begin(),files.end());
            }
        }
    }
}
