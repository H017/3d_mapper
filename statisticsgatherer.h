#ifndef STATISTICSGATHERER_H
#define STATISTICSGATHERER_H

#include <vector>
#include <string>


class StatisticsGatherer
{
public:
    StatisticsGatherer();
    void addFitnessScore(float fs);
    float calculateFitnessScoresMean();
    void addNonConvergedCloud(std::string nonConvergedCloud);

    void saveStaticticsFile();

private:
    std::vector<float> fitnessScores;
    std::vector<std::string> nonConvergedClouds;
};

#endif // STATISTICSGATHERER_H
