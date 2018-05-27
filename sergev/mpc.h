//
// Model predictive controller
//
#ifndef MPC_MPC_H
#define MPC_MPC_H

#include <vector>
#include <eigen3/Eigen/Dense>

using namespace std;

class MPC {
public:

    // constructor
    MPC();

    // destructor
    virtual ~MPC();

    //
    // Solve the model given an initial state and polynomial coefficients.
    //
    // @param state0: initial state of the car in [x, y, psi, v]
    // @param actuator0: initial actuator values in [steering, throttle]
    // @param ptsx: x coordinates (global coordinates) in the reference trajectory
    // @param ptsy: y coordinates (global coordinates) in the reference trajectory
    //
    bool solve(Eigen::VectorXd state0, Eigen::VectorXd actuator0,
               std::vector<double> ptsx, std::vector<double> ptsy);

    // latency getter
    double getLatency();

    // steering getter
    double getSteering();

    // throttle getter
    double getThrottle();

    // ref_x getter
    std::vector<double> getRefx();

    // ref_y getter
    std::vector<double> getRefy();

    // pred_x getter
    std::vector<double> getPredx();

    // pred_y getter
    std::vector<double> getPredy();

private:

    std::vector<double> ref_x_;  // x coordinates of the reference trajectory
    std::vector<double> ref_y_;  // y coordinates of the reference trajectory

    std::vector<double> pred_x_;  // x coordinates of the predicted trajectory
    std::vector<double> pred_y_;  // y coordinates of the predicted trajectory

    Eigen::VectorXd ref_poly_;          // polynomial coefficients for the reference trajectory
    double steering_;           // constant steering angle
    double throttle_;           // constant throttle

    bool is_last_fit_success_;  // an indicator for the success of last fit

    //
    // update the points in the predicted trajectory
    //
    // @param state0: initial state of the car in [x, y, psi, v]
    // @param actuator: initial actuator values in [steering, throttle]
    //
    void updatePred(const Eigen::VectorXd& state0);

    //
    // update the polynomial coefficients of the reference trajectory as well as
    // the points in the reference trajectory
    //
    // @param x: x coordinates (global coordinates) in the reference trajectory
    // @param y: y coordinates (global coordinates) in the reference trajectory
    // @param x0: x coordinate of the origin of the new coordinate system in the old one
    // @param y0: y coordinate of the origin of the new coordinate system in the old one
    // @param psi: orientation of the new coordinate system with respect to the old one
    //
    void updateRef(const std::vector<double>& x, const std::vector<double>& y,
                   double x0, double y0, double psi);
};

#endif // MPC_MPC_H
