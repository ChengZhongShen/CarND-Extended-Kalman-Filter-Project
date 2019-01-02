// test the FusionEKF.cpp, kalman_filter.cpp, tools.cpp
// test on Windows, use vs code, the config file is in .vscode
// use the data file in working folder

#include <iostream>
#include <sstream>
#include <vector>
#include <Eigen/Dense>
#include "measurement_package.h"
#include "FusionEKF.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::ifstream;
using std::istringstream;
using std::string;
using std::vector;

int main()
{
  // set Measurements
  vector<MeasurementPackage> measurement_pack_list;

  // used to compute the RMSE later
  Tools tools;
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  // hardcode input file with laser and radar measurements
  string in_file_name_ = "./data/obj_pose-laser-radar-synthetic-input.txt";
  ifstream in_file(in_file_name_.c_str(),ifstream::in);

  if (!in_file.is_open())
  {
      cout << "Cannot open input file: " << in_file_name_ << endl;
  }

  // set i to get only first 3 measurements, read data to the meas_package, read groud truth to gound_truch
  string line;
  int i = 0;
  while (getline(in_file, line) && (i<=500))
  {
    MeasurementPackage meas_package;

    istringstream iss(line);
    string sensor_type;
    iss >> sensor_type; // reads first element from the current line
    int64_t timestamp;
    if (sensor_type.compare("L") == 0)
    {
      // laser measurement 
      // read measurements
      meas_package.sensor_type_ = MeasurementPackage::LASER;
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
    else if (sensor_type.compare("R") == 0)
    {
      // continue;
      meas_package.sensor_type_ = MeasurementPackage::RADAR;
      meas_package.raw_measurements_ = VectorXd(3);
      float ro;
      float theta;
      float ro_dot;
      iss >> ro;
      iss >> theta;
      iss >> ro_dot;
      meas_package.raw_measurements_ << ro, theta, ro_dot;
      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(meas_package);
    }
    
    // read the groud truth value
    float x_gt;
    float y_gt;
    float vx_gt;
    float vy_gt;
    iss >> x_gt;
    iss >> y_gt;
    iss >> vx_gt;
    iss >> vy_gt;

    VectorXd gt_values(4);
    gt_values(0) = x_gt;
    gt_values(1) = y_gt; 
    gt_values(2) = vx_gt;
    gt_values(3) = vy_gt;
    ground_truth.push_back(gt_values);
    ++i;
  }

  // Create a Kalman Filter instance
  FusionEKF fusionEKF;

  // call the ProcessingMeasurement() function for each measurement
  size_t N = measurement_pack_list.size();
  // start filtering from the second frame
  // (the speed is unknown in the first frame)
  for (size_t k = 0; k < N; ++k)
  {
      fusionEKF.ProcessMeasurement(measurement_pack_list[k]);
  
      // RMSE
      VectorXd estimate(4);
      double p_x = fusionEKF.ekf_.x_(0);
      double p_y = fusionEKF.ekf_.x_(1);
      double v1  = fusionEKF.ekf_.x_(2);
      double v2 = fusionEKF.ekf_.x_(3);

      estimate(0) = p_x;
      estimate(1) = p_y;
      estimate(2) = v1;
      estimate(3) = v2;
      
      // caculated each loops RMSE
      vector<VectorXd> estimations_once;
      vector<VectorXd> ground_truth_once;
      estimations_once.push_back(estimate);
      ground_truth_once.push_back(ground_truth[k]);
      VectorXd RMSE_once = tools.CalculateRMSE(estimations_once, ground_truth_once);


      bool problem = false;
      // check if RMSE > 0.5
      for (int i = 0; i < 4; i ++)
      {
        if (RMSE_once[i] > 1.0)
        {
          problem = true;
        }
      }

      if (problem)
      {
        // cout << "Problem !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"
        //      << "Data Source: " << measurement_pack_list[k].sensor_type_ << "\n"
        //      << "loop number: " << k << "\n"
        //      << "Estimated x_ \n"
        //      << estimate << "\n"
        //      << "RMSE_once\n"
        //      << RMSE_once << "\n";  
      }
      
      // cout << "loop number: " << k << "\n"
      //     << "Estimated x_ \n"
      //     << estimate << "\n"
      //     << "RMSE_once\n"
      //     << RMSE_once << "\n";   

      estimations.push_back(estimate);
  }

  VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);
  cout << "Total RMSE = \n" << RMSE << "\n"; 

  if (in_file.is_open())
  {
      in_file.close();
  }

  return 0;
}