#ifndef MAPPERCONFIG_H
#define MAPPERCONFIG_H

#include <yaml-cpp/yaml.h>

struct CameraConfig
{
    float sigmaS = 10.0f;
    float sigmaR = 0.05f;
};

struct CloudMergerConfig
{
    float transformationEpsilon = 1e-6;
    float maxCorrespondenceDistance = 0.03;
    int maximumIterations = 20;
    float mlsSearchRadiusSmoothing = 0.03f;
    float mlsSearchRadiusFinal = 0.03f;
};

class MapperConfig
{
public:
    MapperConfig();
    static MapperConfig& getInstance()
    {
        static MapperConfig mc;
        return mc;
    }

    const CameraConfig& getCameraConfig();
    const CloudMergerConfig& getCloudMergerConfig();
    const std::string getDirectoryPath();
    bool isLive();

private:
    CameraConfig cameraConfig;
    CloudMergerConfig cloudMergerConfig;
    std::string directoryPath;
    bool live;
};

#endif // MAPPERCONFIG_H
