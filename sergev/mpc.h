//
// Model predictive controller
//
#ifndef MPC_MPC_H
#define MPC_MPC_H

#include <vector>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

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
    bool solve(VectorXd state0, VectorXd actuator0,
               vector<double> ptsx, vector<double> ptsy);

    // latency getter
    double getLatency();

    // steering getter
    double getSteering();

    // throttle getter
    double getThrottle();

    // ref_x getter
    vector<double> getRefx();

    // ref_y getter
    vector<double> getRefy();

    // pred_x getter
    vector<double> getPredx();

    // pred_y getter
    vector<double> getPredy();

private:

    vector<double> ref_x_;      // x coordinates of the reference trajectory
    vector<double> ref_y_;      // y coordinates of the reference trajectory

    vector<double> pred_x_;     // x coordinates of the predicted trajectory
    vector<double> pred_y_;     // y coordinates of the predicted trajectory

    double steering_;           // constant steering angle
    double throttle_;           // constant throttle

    bool is_last_fit_success_;  // an indicator for the success of last fit

    //
    // update the points in the predicted trajectory
    //
    // @param state0: initial state of the car in [x, y, psi, v]
    // @param actuator: initial actuator values in [steering, throttle]
    //
    void updatePred(const VectorXd &state0);

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
    void updateRef(const vector<double> &x, const vector<double> &y,
                   double x0, double y0, double psi);
};

#endif // MPC_MPC_H
