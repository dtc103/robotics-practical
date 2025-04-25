#include "online_statistics/online_statistics.hpp"
#include <cmath>

// https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Welford's_online_algorithm

OnlineStatistics::OnlineStatistics()
{
    this->mean = 0.0;
    this->std = 0.0;
    this->M2 = 0.0;
    this->measurements = 0;
}

void OnlineStatistics::update(double value) {
    this->measurements += 1;
    double delta_old = value - this->mean;
    this->mean += delta_old / this->measurements;

    double delta_new = value - this->mean;
    this->M2 += delta_old * delta_new;

    this->std = std::sqrt(this->M2 / (this->measurements - 1));
}

double OnlineStatistics::getMean() {
    return this->mean;
}

double OnlineStatistics::getStandardDeviation() {
    return this->std;
}

