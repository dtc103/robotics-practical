#include "online_statistics/online_statistics.hpp"
#include <cmath>

// https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Welford's_online_algorithm

OnlineStatistics::OnlineStatistics()
{
    this->mean = 0.0;
    this->std = 0.0;
    this-> M2 = 0;
    this->measurmentes = 0;
}

void OnlineStatistics::update(double value) {
    this->measurements += 1;
    double delta_1 = value - this->mean;
    this->mean += delta_1 / this->measurements;
    double delta_2 = value - this->mean;
    this->M2 += delta_1 * delta_2;
}

double OnlineStatistics::getMean() {
    return this->mean
}

double OnlineStatistics::getStandardDeviation() {
    // unbiased std
    return std::sqrt(this->M2 / (n-1))

    // biased std
    //return std::sqrt(this->M2 / n)
}

