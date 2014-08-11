#include "statisticsgatherer.h"
#include <yaml-cpp/yaml.h>
#include <time.h>
#include <fstream>

StatisticsGatherer::StatisticsGatherer()
{
}

void StatisticsGatherer::addFitnessScore(float fs)
{
    fitnessScores.push_back(fs);
}

void StatisticsGatherer::addNonConvergedCloud(std::string nonConvergedCloud)
{
    nonConvergedClouds.push_back(nonConvergedCloud);
}

float StatisticsGatherer::calculateFitnessScoresMean()
{
    float mean = 0;
    for(int i =0; i<fitnessScores.size();i++)
    {
        mean += fitnessScores[i];
    }

    mean /= (float)fitnessScores.size();

    return mean;
}

void StatisticsGatherer::saveStaticticsFile()
{
    YAML::Emitter out;
    time_t stamp = time(0);



    std::string filename = "mapperstats.yaml";

    out << YAML::BeginMap;
    out << YAML::Key << "Time" << YAML::Value << stamp;
    out << YAML::Key << "NumberOfConvergedClouds" << YAML::Value << fitnessScores.size();
    out << YAML::Key << "NumberOfNonConvergedClouds" << YAML::Value << nonConvergedClouds.size();
    out << YAML::Key << "FitnessScoreMean" << YAML::Value << calculateFitnessScoresMean();
    out << YAML::Key << "NonConvergedClouds" << YAML::Value <<
           YAML::BeginSeq << nonConvergedClouds << YAML::EndSeq;
    out << YAML::EndMap;

    std::ofstream file;
    file.open(filename.c_str(),std::ios::out);
    file << out.c_str();
    file.close();

}
