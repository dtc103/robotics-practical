#pragma once

class OnlineStatistics {
    public:
        OnlineStatistics();

        void update(double);

        double getMean();
        double getStandardDeviation();

    private:
        double mean;
        double std;
        double M2;
        int measurements;
};

