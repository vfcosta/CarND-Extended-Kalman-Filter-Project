#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

class Tools {
public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &ground_truth);

  /**
  * A helper method to calculate Jacobians.
  */
  Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);

  /**
  * A helper method to convert polar to cartesian.
  */
  Eigen::VectorXd ConvertPolarToCartesian(const Eigen::VectorXd& measurements);

  /**
  * A helper method to convert cartesian to polar.
  */
  Eigen::VectorXd ConvertCartesianToPolar(const Eigen::VectorXd& x);

  float epsilon = 0.0001;

};

#endif /* TOOLS_H_ */
