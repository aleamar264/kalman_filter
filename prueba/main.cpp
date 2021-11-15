#include <iostream>
#include <vector>
#include "Eigen/Dense"

using std::cout;
using std::endl;
using std::vector;
using Eigen::VectorXd;
using Eigen::MatrixXd;

//Variables del filtro de kalman
VectorXd x; //objeto estado
MatrixXd P; //matriz de covarianza del objeto
VectorXd u; //fuerza externa
MatrixXd F; //matriz de transicion
MatrixXd H; //matriz de medicion
MatrixXd R; //matriz de covarianza de la medicion
MatrixXd I; //Matriz identidad
MatrixXd Q; //matriz de covarianza del proceso


vector<VectorXd> measurements;

void filter(VectorXd &x, MatrixXd &P);

int main(){
    /**
     * Codigo usado como ejemplo de como trabaja
     * las matrices Eigen
     */

    x = VectorXd(2);
    x << 0, 0;

    P = MatrixXd(2,2);
    P << 1000, 0,
        0, 1000;
    
    u = VectorXd(2);
    u << 0,0;

    F = MatrixXd(2,2);
    F <<1, 1,
        0, 1;
    
    H = MatrixXd(1,2);
    H << 1,
        0;
    
    R = MatrixXd(1,1);
    R << 1;

    I = MatrixXd::Identity(2,2);

    Q = MatrixXd(2,2);
    Q << 0, 0,
        0, 0;
    

    // crear una lista de mediciones
    VectorXd single_means(1);
    single_means << 1;
    measurements.push_back(single_means);
    single_means << 2;
    measurements.push_back(single_means);
    single_means << 3;
    measurements.push_back(single_means);


    // call filter kalman
    filter(x, P);

    return 0;
}

void filter(VectorXd &x, MatrixXd &P){
    
    for (unsigned int n=0; n < measurements.size(); ++n){
        VectorXd z = measurements[n];
        
        // update sate
        VectorXd y = z - H*x;
        MatrixXd HT = H.transpose();
        MatrixXd S = H*P*HT + R;
        MatrixXd Si = S.inverse();
        MatrixXd K = P*HT*Si;

        x = x + (K*y);

        P = (I - K*H)*P;

        //Prediction state

        x = F*x + u;
        MatrixXd Ft = F.transpose();
        P = F*P*Ft + Q;




        cout<< "x=" << endl << x << endl;
        cout<< "P=" << endl << P << endl;

    }


    
}