// conversions.h

#ifndef CONVERSIONS_H
#define CONVERSIONS_H


inline double convert_center_link(double val) {
    double offset = -0.00523598776 * 3.3;

    double min_angle = -0.663225116;
    double max_angle = 0.663225116;
    double min_value = 100.0;
    double max_value = 2781.0;
    double result = ((val - min_value) * (max_angle - min_angle) / (max_value - min_value) + min_angle) + offset;
    return result;
}

inline double convert_boom(double val) {
    // TODO get correct values for these
    double offset = -0.00523598776 * 3.3;

    double min_angle = -0.663225116;
    double max_angle = 0.663225116;
    double min_value = 100.0;
    double max_value = 2781.0;
    double result = ((val - min_value) * (max_angle - min_angle) / (max_value - min_value) + min_angle) + offset;
    return result;
}

inline double convert_bucket(double val) {
    // TODO get correct values for these
    double offset = -0.00523598776 * 3.3;

    double min_angle = -0.663225116;
    double max_angle = 0.663225116;
    double min_value = 100.0;
    double max_value = 2781.0;
    double result = ((val - min_value) * (max_angle - min_angle) / (max_value - min_value) + min_angle) + offset;
    return result;
}


#endif