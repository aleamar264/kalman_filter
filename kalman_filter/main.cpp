#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include "Eigen/Dense"
#include "measurement_package.h"
#include "tracking.h"


using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::ifstream;
using std::istringstream;
using std::vector;
using std::string;

int main(){

    /**
     * @brief Se inicia las mediciones
     */

    vector<MeasurementPackage> measurement_pack_list;
    string in_file_name_ = "obj-pose-lidar-radar-synthetic-input.txt";
    ifstream in_file(in_file_name_.c_str(), ifstream::in);

    if (!in_file.is_open()){
        cout << "No se puede abrir el archivo" << in_file_name_ << endl;
    }
    string line;
     int i = 0;
     while (getline(in_file, line) && (i <= 3)){
         MeasurementPackage meas_package;
         
         istringstream iss(line);
         string sensor_type;
         iss >> sensor_type; // lee el primer elemento de la linea actual
         int64_t timestamp;
        if (sensor_type.compare("L") == 0){ // Lidar measurement
            // read measurement
            meas_package.sensor_type_  = MeasurementPackage::LASER;
            meas_package.raw_measurements_ = VectorXd(2);
            float x;
            float y;
            iss >> x;
            iss >> y;
            meas_package.raw_measurements_ << x, y;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;
            measurement_pack_list.push_back(meas_package);
        }
        else if(sensor_type.compare("R") == 0){ continue;}
        i++;
     }
     Tracking tracking;
     size_t N = measurement_pack_list.size();
    //  start filtering from the second frame
    // the speed is unknown in the fisrt frame
    for (size_t k = 0 ; k < N; ++k) {
        tracking.ProcessMeasurement(measurement_pack_list[k]);
    }
    
    if (in_file.is_open()){
        in_file.close();
    }
    return 0;
}