#include <iostream>
#include <vector>
#include "Eigen/Dense"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;


MatrixXd CalculateJacobian(const VectorXd& x_state);

int main (){

    VectorXd x_predict(4);
    x_predict << 1, 2, 0.2, 0.4;

    MatrixXd Hj = CalculateJacobian(x_predict);
    cout << "Hj:" << endl << Hj << endl;

    return 0;
}


MatrixXd CalculateJacobian( const VectorXd& x_state){
    MatrixXd Hj(3,4);
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);
    float px_2 = px * px;
    float py_2 = py * py;

    if (px == 0|| py == 0||  vx == 0|| vy== 0){
        cout << "No se puede realizar division por zero" << endl;
        return Hj;
    }


    Hj << px/std::sqrt(px_2 + py_2), py/std::sqrt(px_2 + py_2),0 , 0,
        -(py/ (px_2 + py_2)), px/(px_2+py_2), 0, 0,
        py*(vx*py - vy*px)/std::pow(px_2+py_2, 3/2), px*(vy*px - vx*py)/std::pow(px_2+py_2, 3/2), (px)/(std::sqrt(px_2 + py_2)), (py)/(std::sqrt(px_2 +py_2));

    return Hj;
}