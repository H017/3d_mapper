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
    bool isLive();

private:
    CameraConfig cameraConfig;
    CloudMergerConfig cloudMergerConfig;
    bool live;
};

#endif // MAPPERCONFIG_H
