// lowpass_filter.h
#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H


#include <vector> // for a, b and zi
#include <cmath> // for sqrt
#include <algorithm> // for max

class LowpassFilter {

    public:
    LowpassFilter(double freq, double cutoff, double zeta = 1/std::sqrt(2),
                  int order = 1, bool derivator = false, bool prewarp = false);
        double y;
        double freq;
        double cutoff;
        double zeta;
        

        int order;

        bool derivator = false;
        bool prewarp = false;

        double step(double x);

        // test-only accessors
        const std::vector<double>& get_a() const { return a; }
        const std::vector<double>& get_b() const { return b; }


        

    private:
        std::vector<double> a;
        std::vector<double> b;
        std::vector<double> zi;

        std::vector<double> multiply_polynomial(const std::vector<double>& p, const std::vector<double>& q);
        std::vector<double> pow_polynomial(const std::vector<double>& p, int power);
        void bilinear(const std::vector<double>& num, const std::vector<double>& den);
        void design_filter();
};


#endif