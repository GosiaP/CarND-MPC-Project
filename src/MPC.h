#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC 
{
 public:
  MPC();
    virtual ~MPC();

    static void PredictState(
      double const& x0_, double const& y0_, double const& psi0_, double const& v0_, double const& cte0_, double const& epsi0_,
      double& x_, double& y_, double& psi_, double& v_, double& cte_, double& epsi_,
      double a_, double delta);

    static std::pair<Eigen::VectorXd, Eigen::VectorXd> Transform2VehicleCoord(
      std::vector<double> const& ptsx_,
      std::vector<double> const& ptsy_,
      double psi_,
      double px_,
      double py_);

    vector<double> cur_x_vals;
    vector<double> cur_y_vals;

    // This value assumes the model presented in the classroom is used.
    //
    // It was obtained by measuring the radius formed by running the vehicle in the
    // simulator around in a circle with a constant steering angle and velocity on a
    // flat terrain.
    //
    // Lf was tuned until the the radius formed by the simulating the model
    // presented in the classroom matched the previous radius.
    //
    // This is the length from front to CoG that has a similar radius.

    // constants
    static const double Lf;
    static double mph_to_mps_factor;
    static double ref_v;
    static double latency;

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
