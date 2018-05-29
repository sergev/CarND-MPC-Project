//
// Model predictive controller
//
#ifndef MPC_MPC_H
#define MPC_MPC_H

#include <vector>
#include <fstream>

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
    void solve(vector<double> state0, vector<double> actuator0,
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

    unsigned step_;             // index of computation step
    unsigned round_;            // round number
    unsigned last_round_;       // step index of last round

    double init_x_;             // initial starting point, x
    double init_y_;             // initial starting point, y

    ofstream trace_;            // debug output

    //
    // update the points in the predicted trajectory
    //
    // @param state0: initial state of the car in [x, y, psi, v]
    // @param actuator: initial actuator values in [steering, throttle]
    //
    void updatePred(vector<double> state0);

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
    void updateRef(vector<double> x, vector<double> y,
                   double x0, double y0, double psi);
};

#endif // MPC_MPC_H
