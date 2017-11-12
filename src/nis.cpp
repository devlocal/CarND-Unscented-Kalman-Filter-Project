#include <iostream>
#include <fstream>
#include "nis.h"

using namespace ukf;

void Nis::Add(const MatrixXd &prediction, const MatrixXd &measurement, const MatrixXd &S) {
  MatrixXd d = measurement - prediction;
  MatrixXd e_matrix = d.transpose() * S.inverse() * d;
  float e = e_matrix(0, 0);

  data_.push_back(e);
}

void Nis::WriteToFile(const std::string& file_name) {
  std::ofstream of(file_name);
  for (float e: data_) {
    of << e << std::endl;
  }
}