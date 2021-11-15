#ifndef TRACKING_H_
#define TRAKING_H_

#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "measurement_package.h"

class Tracking{
public:
    Tracking();
    virtual ~Tracking();
    void ProcessMeasurement(const MeasurementPackage &measurement_pack);
    KalmanFilter kf_;

private:
    bool is_initialized_;
    int64_t previous_timestamp;

    // componentes de la aceleracion
    float noise_ax;
    float noise_ay;


};
#endif