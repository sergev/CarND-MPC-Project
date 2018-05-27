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
#include "utilities.h"

namespace {
    const double LATENCY    = 0.1;      // latency of the data acquisition system (in seconds)
    const double MAX_SPEED  = 60;       // maximum speed, miles per hour
    const int    POLY_ORDER = 3;        // order of the polynomial fit

    // time step should be long enough to make the system be able to response timely
    const double PRED_TIME_STEP     = 0.2;      // prediction time step in seconds
    const int    N_STEP             = 8;        // number of prediction steps
    const double TRACKING_TIME_STEP = 0.01;     // tracking time step in seconds
    const double MAX_STEERING       = 25 * M_PI/180; // max 25 degrees
    const double MAX_THROTTLE       = 1.0;

    //
    // compute one actuator value after a certain time
    // @param dt: time step
    // @param p: coefficients used in calculation
    //
    template <class T, class Vector>
    T compute_actuator(Vector p, T t, T max_abs_x) {
        T result = p[0] + p[1]*t + p[2]*t*t + p[3]*t*t*t;

        if ( result > max_abs_x ) {
          result = max_abs_x;
        } else if ( result < - max_abs_x) {
          result = -max_abs_x;
        }

        return result;
    };

    using CppAD::AD;

    class FG_eval {
    public:

        typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

        //
        // constructor
        //
        // @param state0: initial state [px, py, psi, v]
        // @param ref_p: coefficients of the polynomial fit for the reference trajectory
        // @param max_speed: maximum speed of the vehicle
        //
        FG_eval(Eigen::VectorXd state0, Eigen::VectorXd ref_poly) {
            state0_ = state0;
            ref_poly_ = ref_poly;
        }

        //
        // Overload () operator.
        //
        // @param fg: objective and constraints
        //            The first one is objective and the rest are constraints.
        // @param x: variables
        //
        void operator()(ADvector& fg, const ADvector& x) {
            // initialize fg
            for (size_t i=0; i < fg.size(); ++i) { fg[i] = 0; }

            AD<double> px0 = state0_[0];
            AD<double> py0 = state0_[1];
            AD<double> psi = state0_[2];

            ADvector next_state(4);
            for (size_t i=0; i<next_state.size(); ++i) { next_state[i] = state0_[i]; }

            ADvector next_actuator(2);
            next_actuator[0] = x[0];    // steering angle
            next_actuator[1] = x[1];    // throttle

            AD<double> cte = 0.0;  // current cross track error
            AD<double> coe = 0.0;  // current orientation error

            AD<double> ss_cte = 0.0;  // sum of square of cross track error
            AD<double> ss_coe = 0.0;  // sum of square of orientation error
            AD<double> ss_speed = 0.0;  // sum of square of speed

            AD<double> max_speed = 0.0;
            AD<double> min_speed = 200.0;
            AD<double> max_abs_cte = 0.0;
            AD<double> max_abs_coe = 0.0;

            AD<double> x_last = 0.0;
            AD<double> y_last = 0.0;

            AD<double> dt = PRED_TIME_STEP;
            AD<double> t = 0;
            for (int i=0; i<N_STEP; ++i) {
                t += dt;

                // computer the next state
                // in each dt, the tracking is further divided into many sub-steps with
                // size TRACKING_TIME_STEP
                AD<double> sub_t = 0.0;
                while ( sub_t < dt - TRACKING_TIME_STEP ) {
                    sub_t += TRACKING_TIME_STEP;
                    next_state = globalKinematic(
                        next_state, next_actuator, (AD<double>)TRACKING_TIME_STEP);
                }
                next_state = globalKinematic(next_state, next_actuator, dt - sub_t);

                // transform from global coordinates system to car's coordinate system
                ADvector X_new = globalToCar<ADvector, AD<double>>(
                    next_state[0], next_state[1], px0, py0, psi);

                // calculate the cross track error`
                cte = X_new[1] - polyEval(ref_poly_, X_new[0]);

                // calculate the orientation error
                if (i > 0) {
                    AD<double> dx = dt*next_state[3];
                    AD<double> dy = polyEval(ref_poly_, X_new[0] + dx/2) -
                                    polyEval(ref_poly_, X_new[0] - dx/2);
                    AD<double> ref_orientation = CppAD::atan(dy/dx);

                    AD<double> current_orientation = CppAD::atan(
                        (X_new[1] - y_last)/(X_new[0] - x_last));

                    x_last = X_new[0];
                    y_last = X_new[1];

                    coe = current_orientation - ref_orientation;
                }

                // update several parameters

                if ( abs(cte) > max_abs_cte ) { max_abs_cte = abs(cte); }
                if ( abs(coe) > max_abs_coe ) { max_abs_coe = abs(coe); }
                if ( next_state[3] > max_speed ) { max_speed = next_state[3]; }
                if ( next_state[3] < min_speed ) { min_speed = next_state[3]; }

                // penalty functions

                ss_speed += next_state[3]*next_state[3];
                ss_cte += cte*cte;
                ss_coe += coe*coe;

                // use the coe in the second step to approximate the one in the first step
                if ( i == 1 ) { ss_coe += coe*coe; }
            }
            // objective function
            std::cout << "ss_cte: " << ss_cte << " "
                      << "ss coe: " << ss_coe << " "
                      << "ss speed: " << ss_speed << std::endl;

            fg[0] += 0.002*ss_cte + ss_coe;

            // speed control
            if (state0_[3] < MAX_SPEED) {
                fg[0] -= (MAX_SPEED - state0_[3]) * 1e-6 * ss_speed;
            }

            // constraint functions
            fg[1] = max_abs_cte;
            return;
        }

    private:

        Eigen::VectorXd state0_;  // initial state of the optimization
        Eigen::VectorXd ref_poly_;  // coefficients of the polynomial fit for the reference trajectory

    };
}

// Implement MPC

MPC::MPC() {
    ref_poly_ = Eigen::VectorXd::Zero(POLY_ORDER + 1);

    ref_x_ = std::vector<double>(20);
    ref_y_ = std::vector<double>(20);

    pred_x_ = std::vector<double>(N_STEP);
    pred_y_ = std::vector<double>(N_STEP);

    is_last_fit_success_ = true;
}

MPC::~MPC() {}

double MPC::getLatency() { return LATENCY; }

// the steering value should be normalized to [-1, 1]
double MPC::getSteering() { return steering_ / MAX_STEERING; }

double MPC::getThrottle() { return throttle_; }

std::vector<double> MPC::getRefx() { return ref_x_; }

std::vector<double> MPC::getRefy() { return ref_y_; }

std::vector<double> MPC::getPredx() { return pred_x_; }

std::vector<double> MPC::getPredy() { return pred_y_; }

void MPC::updatePred(const Eigen::VectorXd& state0) {
    Eigen::VectorXd next_state(4);
    for (unsigned i=0; i<next_state.size(); ++i) { next_state[i] = state0[i]; }

    Eigen::VectorXd next_actuator(2);
    next_actuator[0] = steering_;
    next_actuator[1] = throttle_;

    double dt = PRED_TIME_STEP;  // set time step
    double t = 0;
    for (int i=0; i<N_STEP; ++i) {
        t += dt;

        double sub_t = 0.0;
        while ( sub_t < dt - TRACKING_TIME_STEP ) {
            sub_t += TRACKING_TIME_STEP;
            next_state = globalKinematic(
                next_state, next_actuator, TRACKING_TIME_STEP);
        }
        next_state = globalKinematic(next_state, next_actuator, dt - sub_t);

        // transform from global coordinates system to car's coordinate system
        std::vector<double> X_new = globalToCar<std::vector<double>, double>(
            next_state[0], next_state[1], state0[0], state0[1], state0[2]);

        // assign reference trajectory for each step
        pred_x_[i] = X_new[0];
        pred_y_[i] = X_new[1];
    }
}

void MPC::updateRef(const std::vector<double>& x, const std::vector<double>& y,
                    double px0, double py0, double psi) {
    assert(x.size() == y.size());

    size_t length0 = x.size();

    Eigen::VectorXd ref_x(length0);
    Eigen::VectorXd ref_y(length0);

    // 1. Fit the read out reference trajectory

    for (size_t i=0; i<x.size(); ++i) {
        // transform from global coordinates system to car's coordinate system
        std::vector<double> X_new = globalToCar<std::vector<double>, double>(
            x[i], y[i], px0, py0, psi);
        // store the reference trajectory in the car's coordinate system
        ref_x[i] = X_new[0];
        ref_y[i] = X_new[1];
    }

    // polynomial fit of the reference trajectory
    ref_poly_ = leastSquareFit(ref_x, ref_y, POLY_ORDER);

    // 2. Generate a finer reference trajectory

    double x_max = ref_x[ref_x.size() - 1];
    long ref_length = ref_x_.size();
    double x_step = x_max / ref_length;

    for (int i=0; i<ref_length; ++i) {
        ref_x_[i] = x_step*i;
        ref_y_[i] = polyEval(ref_poly_, ref_x_[i]);
    }
}

bool MPC::solve(Eigen::VectorXd state0, Eigen::VectorXd actuator0,
                std::vector<double> ptsx, std::vector<double> ptsy)
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
    Eigen::VectorXd estimated_state0(4);
    for (int i=0; i<state0.size(); ++i) { estimated_state0[i] = state0[i]; }

    double sub_t = 0.0;
    while ( sub_t < LATENCY - TRACKING_TIME_STEP ) {
        sub_t += TRACKING_TIME_STEP;
        estimated_state0 = globalKinematic(
            estimated_state0, actuator0, TRACKING_TIME_STEP);
    }
    estimated_state0 = globalKinematic(estimated_state0, actuator0, LATENCY - sub_t);

    //
    // set up the optimizer
    //
    typedef CPPAD_TESTVECTOR(double) Dvector;

    // number of variables:
    size_t nx = 2;
    Dvector xi(nx), xl(nx), xu(nx);

    // initialize variables
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

    // number of constraints:
    size_t ng = 1;
    Dvector gl(ng), gu(ng);

    // set constraint boundaries
    // It is easy to get an evil fit with multiple constraints

    // constraint 1: maximum absolute cte
    gl[0] = 0.0;
    gu[0] = 2.0;

    // object that computes objective and constraints
    FG_eval fg_eval(estimated_state0, ref_poly_);

    // options for IPOPT solver
    std::string options;

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

    is_last_fit_success_ = true;
    is_last_fit_success_ &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

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

    // Print out the results
    std::cout << "solution status: " << solution.status << std::endl;
    std::cout << "Cost " << solution.obj_value << std::endl;
    std::cout << "optimized variables: ";
    for (unsigned i=0; i<solution.x.size(); ++i) {
        std::cout << solution.x[i] << "  ";
    }
    std::cout << std::endl;

    std::cout << "constraints: ";
    for (unsigned i=0; i<solution.g.size(); ++i) {
        std::cout << solution.g[i] << "  ";
    }
    std::cout << std::endl;

    // assign the optimized values
    steering_ = solution.x[0];
    throttle_ = solution.x[1];

    // update the predicted trajectory
    updatePred(estimated_state0);

    return is_last_fit_success_;
}
