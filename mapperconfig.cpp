#include "mapperconfig.h"

MapperConfig::MapperConfig()
{
    YAML::Node config = YAML::LoadFile("mapper_config.yaml");
    YAML::Node cameraNode = config["camera"];
    cameraConfig.sigmaR = cameraNode["sigma_r"].as<float>();
    cameraConfig.sigmaS = cameraNode["sigma_s"].as<float>();

    YAML::Node cloudMergerNode = config["cloud_merger"];
    cloudMergerConfig.maxCorrespondenceDistance = cloudMergerNode["max_correspondence_distance"].as<float>();
    cloudMergerConfig.transformationEpsilon = cloudMergerNode["transformation_epsilon"].as<float>();
    cloudMergerConfig.maximumIterations = cloudMergerNode["maximum_iterations"].as<int>();

    live = config["live"].as<bool>();

}

const CameraConfig& MapperConfig::getCameraConfig()
{
    return cameraConfig;
}

const CloudMergerConfig& MapperConfig::getCloudMergerConfig()
{
    return cloudMergerConfig;
}

bool MapperConfig::isLive()
{
    return live;
}
