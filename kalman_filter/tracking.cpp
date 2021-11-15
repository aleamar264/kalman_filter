#include "tracking.h"
#include <iostream>
#include "Eigen/Dense"
#include <vector>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

Tracking::Tracking(){
    is_initialized_ = false;
    previous_timestamp = 0;

    // Se crea un vector de 4D
    kf_.x_ = MatrixXd(4);
    // covarianza P
    kf_.P_ = MatrixXd(4,4);
    kf_.P_ << 1, 0, 0, 0,
            0, 1, 0 , 0,
            0, 0 , 1000, 0,
            0, 0 ,0 , 1000;
    
    // covarianza de la medicion
    kf_.R_ = MatrixXd(2, 2);
    kf_.R << 0.0225, 0,
            0, 0.0225;

    // matriz de mnedicion
    kf_.H_ = MatrixXd(2,4);
    kf_.H_ =  << 1, 0, 0, 0,
                0, 1, 0, 0;
    
    // matriz de transicion F
    kf_.F_ = MatrixXd(4,4);
    kf_.F_<< 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0.
            0, 0, 0, 1;

    // componentes de ruido aceleracion
    noise_ax = 5;
    noise_ay = 5;    
}

Traking::~Tracking(){}

// Proceso de un solo paquete de medicion[]
void Tracking::ProcessMeasurement(const MeasurementPackage &measurement_pack){
    if(!is_initialized_){
        cout << "Kalman Filter initialization" << endl;
        kf_.x_ = << measurement_pack.raw_measurements_[0],
                    measurement_pack.raw_measurements_[1],
                    0,
                    0;

        previous_timestamp = measurement_pack.timestamp_;
        is_initalized = true;
        return ;
        }

    // se calcula el timepo pasado entre los tiempos
    // expresado en segundos
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = measurement_pack.timestamp_;
    kf_.F_(0,2) = dt;
    kf_.F_(1,3) = dt;
    float dt_2 = dt * dt;
    float dt_3 = dt_2 * dt;
    float dt_4 = dt_3 * dt;

    kf_.Q_ = MatrixXd(4, 4);
    kf_.Q_ << (dt_4/4)*noise_ax, 0, ((dt_3)/2)*noise_ax, 0,
        0, ((dt_4)/4)*noise_ay, 0, ((dt_3)/2)*noise_ay,
        (dt_3)/2)*noise_ax, 0, (dt_2)*noise_ax, 0,
        0, ((dt_3)/2)*noise_ay, 0, (dt_2)*noise_ay;

    kf_.Predict();
    kf_.Update(measurement_pack.raw_measuerements);

    cout << "X=" << kf_.x_ << endl;
    cout << "P_" << kf_.P_ << endl;
    

    }
