#ifndef KALMAN_FILTER_H_
#define KALMNA_FILTER_H_

#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class KalmanFilter {
public:
    /**
     * @brief constructor
     */
    KalmanFilter();
    /**
     * @brief Destructor
     */
    virtual ~KalmanFilter();
    /**
     * @brief Predicts Predice el estado y la covarianza de
     * del estado usando el modelo de proceso
     */
    void Predict();
    /**
     * @brief Update: Actualiz el estado y @param z. La medicion es en
     * k+1
     */
    void Update(const VectorXd &z);

    // vector de estado
    VectorXd x_;

    // Matriz de covarianza 
    MatrixXd P_;

    // Matriz de transicion de estado
    MatrixXd F_;

    // Matriz de covarianza de proceso
    MatrixXd Q_;

    // matriz de mediciom
    MatrixXd H_;

    // Matriz de covarianza de la medicion
    MatrixXd R_;
        
};

#endif
