#include <iostream>
#include <math.h>
#include "Eigen/Dense"
#include <vector>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::endl;
using std::cout;
using std::vector;

VectorXd CalculateRMSE(const vector<VectorXd> &estimatiosn, 
                        const vector<VectorXd> &ground_truth);

int main(){


    vector<VectorXd> estimation;
    vector<VectorXd> ground_truth;

    // estimaciones
    VectorXd e(4);
    e << 1, 2, 0.2, 0.1;
    estimation.push_back(e);
    e << 2, 2, 0.3, 0.2;
    estimation.push_back(e);
    e << 3, 3, 0.4, 0.3;
    estimation.push_back(e);

    // valores verdaderos

    VectorXd g(4);
    g << 1.1, 1.1, 0.3, 0.2;
    ground_truth.push_back(g);
    g << 2.1, 2.1, 0.4, 0.3;    
    ground_truth.push_back(g);
    g << 3.1, 3.1, 0.5, 0.4;
    ground_truth.push_back(g);

    cout << CalculateRMSE(estimation, ground_truth) << endl;

    return 0;
}

VectorXd CalculateRMSE(const vector<VectorXd> &estimations, 
                        const vector<VectorXd> &ground_truth){

    VectorXd rmse(4);
    rmse << 0,0,0,0;

    if (estimations.size() == 0  || ground_truth.size() == 0){
        cout << "Las dimensiones de los vectores estimations y ground_truth son 0 " << endl;
        return rmse;
    }

    

    for(int i =0; i < estimations.size(); i ++){
        VectorXd residuals = estimations[i] - ground_truth[i];
        residuals = residuals.array()*residuals.array();
        rmse += residuals;
    }

    rmse = rmse/estimations.size();
    rmse = rmse.array().sqrt();

    
    return rmse;
}