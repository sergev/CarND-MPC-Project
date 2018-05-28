//
// Model predictive controller
//
// TODO: How to handle evil fit?
// TODO: Why the fit is prone to become evil when the PRED_TIME_STEP becomes small
//
#include <iostream>

#define HAVE_CSTDDEF
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

#include "mpc.h"

const double LATENCY    = 0.1;      // latency of the data acquisition system (in seconds)
const double MAX_SPEED  = 60;       // maximum speed, miles per hour
const int    POLY_ORDER = 3;        // order of the polynomial fit

// time step should be long enough to make the system be able to response timely
const double PRED_TIME_STEP     = 0.2;      // prediction time step in seconds
const int    N_STEP             = 8;        // number of prediction steps
const double MAX_STEERING       = 25 * M_PI/180; // max 25 degrees
const double MAX_THROTTLE       = 1.0;

using CppAD::AD;

//
// the global kinematic model.
//
// @param state: state of the car in [x, y, psi, v]
// @param actuator: actuator values in [steering, throttle]
// @param dt: time step (s)
//
// @return next_state: state of the car after time dt
//
template <class Vector, class T>
inline Vector globalKinematic(const Vector& state, const Vector& actuator, T dt) {

    // distance between the front wheel and the vehicle center
    // This value was obtained by measuring the radius formed by running the vehicle in the
    // simulator around in a circle with a constant steering angle and velocity on a
    // flat terrain.
    //
    // Lf was tuned until the radius formed by the simulating the model
    // presented in the classroom matched the previous radius.
    //
    // This is the length from front to CoG that has a similar radius.
    const double LF = 2.67;

    // Acceleration in meters per sec^2 at maximum throttle.
    const double MAX_ACCEL = 3.0; //TODO

    // Conversion from MPH to meters per second.
    const double MPH_TO_METERS_PER_SEC = 0.44704;
    const double METERS_PER_SEC_TO_MPH = 2.23694;

    auto px       = state[0];
    auto py       = state[1];
    auto psi      = state[2];
    auto v        = state[3] * MPH_TO_METERS_PER_SEC;
    auto steering = actuator[0];
    auto throttle = actuator[1];

    Vector next_state(state.size());
    next_state[0] = px  + (v * cos(psi) * dt);
    next_state[1] = py  + (v * sin(psi) * dt);
    next_state[2] = psi - (v * steering/LF * dt);
    next_state[3] = (v + (throttle * MAX_ACCEL * dt)) * METERS_PER_SEC_TO_MPH;

    return next_state;
}

//
// transform coordinate from one coordinate system to the other
//
// @param X: coordinates (x, y) in the old coordinate system to be transformed
// @param X0: origin (x0, y0) of the new coordinate system in the old one
// @param psi: orientation of the new coordinate system with respect to the old one
//
template <class Vector, class T>
inline Vector globalToCar(T px, T py, T px0, T py0, T psi) {
    Vector X_new(2);

    T c = cos(psi);
    T s = sin(psi);
    T dx = px - px0;
    T dy = py - py0;

    // a rotation clock-wise about the origin
    X_new[0] = dx*c + dy*s;
    X_new[1] = -dx*s + dy*c;

    return X_new;
}

class FG_eval {
public:

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

    //
    // constructor
    //
    // @param state0: initial state [px, py, psi, v]
    // @param ref_p: coefficients of the polynomial fit for the reference trajectory
    //
    FG_eval(VectorXd state0, vector<double> ref_x, vector<double> ref_y) {
        state0_ = state0;
        ref_x_ = ref_x;
        ref_y_ = ref_y;
    }

    //
    // Overload () operator.
    //
    // @param fg: objective and constraints
    //            The first one is objective and the rest are constraints.
    // @param x: variables
    //
    void operator()(ADvector &objectives, const ADvector &vars) {

        // initialize objectives
        for (unsigned i=0; i < objectives.size(); ++i) {
            objectives[i] = 0;
        }

        AD<double> px0 = state0_[0];
        AD<double> py0 = state0_[1];
        AD<double> psi = state0_[2];

        ADvector next_state(4);
        for (unsigned i=0; i<next_state.size(); ++i) {
            next_state[i] = state0_[i];
        }

        ADvector next_actuator(2);
        next_actuator[0] = vars[0];     // steering angle
        next_actuator[1] = vars[1];     // throttle

        AD<double> cte;                 // current cross track error
        AD<double> ss_cte      = 0.0;   // sum of square of cross track error
        AD<double> ss_speed    = 0.0;   // sum of square of speed
        AD<double> max_abs_cte = 0.0;
        AD<double> t           = 0;

        for (int i=0; i<N_STEP; ++i) {
            t += PRED_TIME_STEP;

            // compute the next state
            next_state = globalKinematic(next_state, next_actuator, PRED_TIME_STEP);

            // transform from global coordinates system to car's coordinate system
            ADvector xy = globalToCar<ADvector, AD<double>>(
                next_state[0], next_state[1], px0, py0, psi);

            // calculate the cross track error
            cte = distanceToRefTrajectory(xy[0], xy[1]);

            // update several parameters
            if (abs(cte) > max_abs_cte) {
                max_abs_cte = abs(cte);
            }

            // penalty functions
            ss_speed += next_state[3] * next_state[3];
            ss_cte += cte * cte;
        }
#if 0
        cout << "ss_cte: " << ss_cte << " "
             << "ss speed: " << ss_speed << endl;
#endif
        // objective function
        objectives[0] += 0.002 * ss_cte;

        // speed control
        if (state0_[3] < MAX_SPEED) {
            objectives[0] -= (MAX_SPEED - state0_[3]) * 1e-6 * ss_speed;
        }

        // constraint functions
        objectives[1] = max_abs_cte;
        return;
    }

private:

    VectorXd       state0_; // initial state of the optimization
    vector<double> ref_x_;  // reference trajectory
    vector<double> ref_y_;

    //
    // Compute minimal distance from a dot to a reference trajectory
    //
    AD<double> distanceToRefTrajectory(AD<double> x, AD<double> y) {
        //TODO:
        // abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2*y1 - y2*x1) /
        //     sqrt((y2 - y1)^2 + (x2 - x1)^2))
        return 1;
    }
};

//
// Implement MPC
//
MPC::MPC() {
    pred_x_ = vector<double>(N_STEP);
    pred_y_ = vector<double>(N_STEP);

    is_last_fit_success_ = true;
}

MPC::~MPC() {}

double MPC::getLatency() { return LATENCY; }

// the steering value should be normalized to [-1, 1]
double MPC::getSteering() { return steering_ / MAX_STEERING; }

double MPC::getThrottle() { return throttle_; }

vector<double> MPC::getRefx() { return ref_x_; }

vector<double> MPC::getRefy() { return ref_y_; }

vector<double> MPC::getPredx() { return pred_x_; }

vector<double> MPC::getPredy() { return pred_y_; }

void MPC::updatePred(const VectorXd &state0) {
    VectorXd next_state(4);
    for (unsigned i=0; i<next_state.size(); ++i) { next_state[i] = state0[i]; }

    VectorXd next_actuator(2);
    next_actuator[0] = steering_;
    next_actuator[1] = throttle_;

    double t = 0;
    for (int i=0; i<N_STEP; ++i) {
        t += PRED_TIME_STEP;
        next_state = globalKinematic(next_state, next_actuator, PRED_TIME_STEP);

        // transform from global coordinates system to car's coordinate system
        vector<double> xy = globalToCar<vector<double>, double>(
            next_state[0], next_state[1], state0[0], state0[1], state0[2]);

        // assign reference trajectory for each step
        pred_x_[i] = xy[0];
        pred_y_[i] = xy[1];
    }
}

void MPC::updateRef(const vector<double>& x, const vector<double>& y,
                    double px0, double py0, double psi) {
    assert(x.size() == y.size());

    unsigned ref_length = x.size();

    // allocate vectors
    ref_x_.resize(ref_length);
    ref_y_.resize(ref_length);

    // transform from global coordinates system to car's coordinate system
    for (unsigned i=0; i<ref_length; ++i) {

        vector<double> xy = globalToCar<vector<double>, double>(
            x[i], y[i], px0, py0, psi);

        // store the reference trajectory in the car's coordinate system
        ref_x_[i] = xy[0];
        ref_y_[i] = xy[1];
    }
}

bool MPC::solve(VectorXd state0, VectorXd actuator0,
                vector<double> ptsx, vector<double> ptsy)
{
    //
    // update the reference trajectory
    // the reference trajectory should be calculated using the measured state
    // vector since the way points and vehicle state were measured at the same
    // time prior to the current time.
    //
    updateRef(ptsx, ptsy, state0[0], state0[1], state0[2]);

    //
    // estimate the current status to compensate the latency
    //
    VectorXd estimated_state0(4);
    for (int i=0; i<state0.size(); ++i) {
        estimated_state0[i] = state0[i];
    }
    estimated_state0 = globalKinematic(estimated_state0, actuator0, LATENCY);

    //
    // set up the optimizer
    //
    typedef CPPAD_TESTVECTOR(double) Dvector;

    //
    // set variables
    //
    Dvector xi(2), xl(2), xu(2);

    if (is_last_fit_success_) {
        xi[0] = steering_;
        xi[1] = throttle_;
    } else {
        xi[0] = actuator0[0];
        xi[1] = actuator0[1];
    }

    // set variable boundaries
    xl[0] = -MAX_STEERING; xu[0] = MAX_STEERING;
    xl[1] = 0;             xu[1] = MAX_THROTTLE;

    //
    // set constraints
    //
    Dvector gl(1), gu(1);

    // set constraint boundaries
    // constraint 1: maximum absolute cte
    gl[0] = 0.0;
    gu[0] = 2.0;

    // object that computes objective and constraints
    FG_eval fg_eval(estimated_state0, ref_x_, ref_y_);

    // options for IPOPT solver
    string options;

    options += "Integer print_level  0\n";
    // Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    options += "Numeric max_cpu_time          0.5\n";

    // structure that holds the solution of the optimization
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(options, xi, xl, xu, gl, gu, fg_eval, solution);

    is_last_fit_success_ = (solution.status ==
                            CppAD::ipopt::solve_result<Dvector>::success);

    // Check some of the solution values
    //      possible values for the result status
    //      enum status_type {
    //           0: not_defined,
    //           1: success,
    //           2: maxiter_exceeded,
    //           3: stop_at_tiny_step,
    //           4: stop_at_acceptable_point,
    //           5: local_infeasibility,
    //           6: user_requested_stop,
    //           7: feasible_point_found,
    //           8: diverging_iterates,
    //           9: restoration_failure,
    //           10: error_in_step_computation,
    //           11: invalid_number_detected,
    //           12: too_few_degrees_of_freedom,
    //           13: internal_error,
    //           14: unknown
    //      };
#if 0
    // Print out the results
    cout << "solution status: " << solution.status << endl;
    cout << "Cost " << solution.obj_value << endl;
    cout << "optimized variables: ";
    for (unsigned i=0; i<solution.x.size(); ++i) {
        cout << solution.x[i] << "  ";
    }
    cout << endl;

    cout << "constraints: ";
    for (unsigned i=0; i<solution.g.size(); ++i) {
        cout << solution.g[i] << "  ";
    }
    cout << endl;
#endif
    // assign the optimized values
    steering_ = solution.x[0];
    throttle_ = solution.x[1];

    // update the predicted trajectory
    updatePred(estimated_state0);

    return is_last_fit_success_;
}
