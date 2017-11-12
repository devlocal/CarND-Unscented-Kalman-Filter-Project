#ifndef UNSCENTEDKF_NIS_H_H
#define UNSCENTEDKF_NIS_H_H

#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace ukf {

class Nis {
public:
  void Add(const MatrixXd &prediction, const MatrixXd &measurement, const MatrixXd &S);
  const std::vector<float>& GetData() const { return data_; }
  void WriteToFile(const std::string& file_name);

private:
  std::vector<float> data_;
};

}
#endif //UNSCENTEDKF_NIS_H_H
