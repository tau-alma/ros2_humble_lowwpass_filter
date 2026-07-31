// lowpass_filter.cpp

#include "lowpass_filter.h"

LowpassFilter::LowpassFilter(double freq, double cutoff, double zeta,
                             int order, bool derivator, bool prewarp)
                            : freq(freq), cutoff(cutoff), zeta(zeta),
                              order(order), derivator(derivator), prewarp(prewarp) {
                                design_filter();
                                // resize zi to be correct size
                                zi.resize(this->order, 0.0);
                            }

std::vector<double> LowpassFilter::multiply_polynomial(const std::vector<double>& p, const std::vector<double>& q) {
    std::vector<double> result(p.size() + q.size() - 1, 0.0);

    // multiply polynomials
    // size() return size_t. -> i and j have same type so no warnings or errors
    for (size_t i = 0; i < p.size(); i++) {
        for (size_t j = 0; j < q.size(); j++) {
            result[i + j] += p[i] * q[j];
        }
    }
    return result;
}

std::vector<double> LowpassFilter::pow_polynomial(const std::vector<double>& p, int power) {
    std::vector<double> result = {1.0};
    for (int i = 0; i < power; i++) {
        result = multiply_polynomial(result, p);
    }
    return result;
}

void LowpassFilter::bilinear(const std::vector<double>& num, const std::vector<double>& den) {
    
    double fac = std::sqrt(2 * freq);

    std::vector<double> zm1 = {-1 * fac, 1 * fac}; // (z-1) * fac
    std::vector<double> zp1 = {1 / fac, 1 / fac}; // (z+1) / fac

    size_t N = std::max(num.size(), den.size()) - 1;

    std::vector<double> b_acc(N + 1, 0.0);
    std::vector<double> a_acc(N + 1, 0.0);

    // Numerator coefficients loop
    for (size_t k = 0; k < num.size(); k++ ) {
        std::vector<double> zm1_k = pow_polynomial(zm1, k); // ((z-1) * fac) ^ k
        std::vector<double> zp1_nk = pow_polynomial(zp1, N - k); // ((z+1) * fac) ^ (N - k)
        std::vector<double> term = multiply_polynomial(zm1_k, zp1_nk); // ((z-1) * fac) ^ k * ((z+1) * fac) ^ (N - k)
        for (size_t i = 0; i < N + 1; i++) {
            b_acc[i] += num[k] * term[i];
        }
    }

    // Denominator coefficients loop
    for (size_t k = 0; k < den.size(); k++ ) {
        std::vector<double> zm1_k = pow_polynomial(zm1, k); // ((z-1) * fac) ^ k
        std::vector<double> zp1_nk = pow_polynomial(zp1, N - k); // ((z+1) * fac) ^ (N - k)
        std::vector<double> term = multiply_polynomial(zm1_k, zp1_nk); // ((z-1) * fac) ^ k * ((z+1) * fac) ^ (N - k)
        for (size_t i = 0; i < N + 1; i++) {
            a_acc[i] += den[k] * term[i];
        }
    }

    // normalize a and b coefficients
    double a0 = a_acc[N];

    for (size_t i = 0; i < b_acc.size(); i++) {
        b_acc[i] = b_acc[i] / a0;
    }

    
    for (size_t i = 0; i < a_acc.size(); i++) {
        a_acc[i] = a_acc[i] / a0;
    }

    // Reverse order of vectors to match the convention of nth power is nth element
    std::reverse(b_acc.begin(), b_acc.end());
    std::reverse(a_acc.begin(), a_acc.end());
    
    this->b = b_acc;
    this->a = a_acc;
}

void LowpassFilter::design_filter() {
    double omega_c = 0.0;
    if (prewarp) {
        omega_c = 2 * freq * std::tan(M_PI * cutoff / freq);
    } else {
        omega_c = 2 * M_PI * cutoff;
    }

    std::vector<double> num;
    std::vector<double> den;

    // vector convention is that nth power is nth element
    if (order == 1) {
        if (derivator) {
            num = {0.0, omega_c};
            den = {omega_c, 1.0};
        } else {
            num = {omega_c};
            den = {omega_c, 1.0};
        }
    } else {
        if (derivator) {
            num = {0.0, omega_c * omega_c};
            den = {omega_c * omega_c, 2.0 * zeta * omega_c, 1.0};
        } else {
            num = {omega_c * omega_c};
            den = {omega_c * omega_c, 2.0 * zeta * omega_c, 1.0};
        }
    }
    bilinear(num, den);
}



double LowpassFilter::step(double x) {
    y = b[0] * x + zi[0];

    if (order == 1) {
        zi[0] = b[1] * x - a[1] * y    ;    
    } else {
        zi[0] = b[1] * x - a[1] * y + zi[1];
        zi[1] = b[2] * x - a[2] * y;
    }

    return y;
}