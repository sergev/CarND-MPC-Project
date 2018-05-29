//
// Model predictive controller
//
#define HAVE_CSTDDEF
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

#include "mpc.h"

const double LATENCY    = 0.1;      // latency of the data acquisition system (in seconds)
const double MAX_SPEED  = 90;       // maximum speed, miles per hour

// time step should be long enough to make the system be able to response timely
const double PRED_TIME_STEP     = 0.1;              // prediction time step in seconds
const int    N_STEP             = 15;               // number of prediction steps
const double MAX_STEERING       = 25 * M_PI/180;    // max 25 degrees
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
//  const double LF = 2.67; // 29.5s 26.9s - original
//  const double LF = 3.2;  // 28.3s 24.8s
    const double LF = 3.4;  // 27.5s 24.3s 24.4s 23.7s 23.7s 23.9s 24.3s 24.8s 24.4s 24.2s 24.4s (26.90)
//  const double LF = 3.5;  // 26.8s 23.8s 24.5s 24.7s 24.4s 23.6s 23.3s 24.2s 24.3s 24.5s 24.7s (26.89)
//  const double LF = 3.6;  // 27.2s 23.1s 23.9s 23.5s 24.4s 24.0s 23.9s 23.7s 24.0s 24.1s 24.0s (26.51)
//  const double LF = 3.7;  // 27.4s 23.5s 24.0s 24.4s 24.6s 23.6s 24.2s 23.8s 23.8s 24.1s 24.4s (26.71)
//  const double LF = 3.8;  // 27.3s 23.4s

    // Acceleration in meters per sec^2 at maximum throttle.

    // Conversion from MPH to meters per second.
    const double MPH_TO_METERS_PER_SEC = 0.44704;
    const double METERS_PER_SEC_TO_MPH = 2.23694;

    auto px       = state[0];
    auto py       = state[1];
    auto psi      = state[2];
    auto v        = state[3] * MPH_TO_METERS_PER_SEC;
    auto steering = actuator[0];
    auto throttle = actuator[1];

    // Acceleration in meters per sec^2 at maximum throttle.
//  const double accel = (throttle >= 0) ? 4.0 : 15.0;  // 26.7s 23.4s 23.5s 23.5s 23.7s 23.7s 23.6s 24.0s 23.5s 23.8s 23.7s
    const double accel = (throttle >= 0) ? 6.0 : 20.0;  // 26.3s 23.3s 22.5s 23.6s 23.2s 22.9s 23.1s 23.0s 23.4s 23.9s 23.4s
//  const double accel = (throttle >= 0) ? 8.0 : 20.0;  // 27.3s 23.4s 23.9s 24.1s 23.9s 24.3s 23.3s 23.9s 23.7s 24.5s 24.5s
//  const double accel = (throttle >= 0) ? 10.0 : 20.0; // 26.9s 24.2s 22.7s 23.5s 23.5s 23.6s 23.9s 23.8s 23.3s 23.8s 23.8s
//  const double accel = (throttle >= 0) ? 14.0 : 20.0; // 27.1s 23.4s 23.4s 24.3s 24.1s 24.1s 23.0s 24.9s 24.3s 24.4s 23.7s
//  const double accel = (throttle >= 0) ? 17.0 : 20.0; // 28.2s 24.2s 24.1s 24.1s 24.8s 23.9s 24.0s 23.6s 24.5s 24.2s 24.8s
//  const double accel = (throttle >= 0) ? 10.0 : 30.0; // 28.2s 24.2s 24.1s 24.1s 24.8s 23.9s 24.0s 23.6s 24.5s 24.2s 24.8s
//  const double accel = (throttle >= 0) ? 6.0 : 30.0;  // 27.0s 23.9s 24.3s 24.1s 23.6s 23.9s 24.1s 24.7s 23.6s 24.1s 24.3s
//  const double accel = (throttle >= 0) ? 15.0 : 45.0; // 27.1s 24.4s 24.1s 25.0s 24.5s 24.1s 24.8s 24.2s 24.9s 24.7s 24.2s

    Vector next_state(state.size());
    next_state[0] = px  + (v * cos(psi) * dt);
    next_state[1] = py  + (v * sin(psi) * dt);
    next_state[2] = psi - (v * steering/LF * dt);
    next_state[3] = (v + (throttle * accel * dt)) * METERS_PER_SEC_TO_MPH;

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
    FG_eval(vector<double> state0, vector<double> ref_x, vector<double> ref_y) {
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

            if (beyondTrajectory(xy[0], xy[1])) {
                // Predicted point is beyond the reference trajectory
                //cout << "\nbeyond trajectory!" << endl;
                break;
            }

            // calculate the cross track error
            cte = distanceToRefTrajectory(xy[0], xy[1]);

            // update several parameters
            if (abs(cte) > max_abs_cte) {
                max_abs_cte = abs(cte);
            }
#if 0
            cout << "cte: " << cte << " "
               << "speed: " << next_state[3] << endl;
#endif
            // penalty functions
            objectives[0] += cte * cte * t;
            ss_speed += next_state[3] * next_state[3];
        }
#if 0
        cout << "penalty: " << objectives[0] << " "
           << "ss_ speed: " << ss_speed << endl;
#endif
        // speed control
        if (state0_[3] < MAX_SPEED) {
            objectives[0] -= (MAX_SPEED - state0_[3]) * 1e-4 * ss_speed;
        }

        // constraint functions
        objectives[1] = max_abs_cte;
        return;
    }

private:

    vector<double> state0_; // initial state of the optimization
    vector<double> ref_x_;  // reference trajectory
    vector<double> ref_y_;

    //
    // Compute distance from a point to a line segment
    //
    AD<double> distanceToLineSegment(
        AD<double> x,  AD<double> y,    // point C
        AD<double> x1, AD<double> y1,   // segment end A
        AD<double> x2, AD<double> y2)   // segment end B
    {
        AD<double> dx = x2 - x1;
        AD<double> dy = y2 - y1;
        AD<double> dx1 = x1 - x;
        AD<double> dy1 = y1 - y;
        AD<double> dx2 = x2 - x;
        AD<double> dy2 = y2 - y;

        // Dot product AB*AC is positive, when the angle is acute.
        AD<double> dot_product_ab_ac = - dx*dx1 - dy*dy1;
        if (dot_product_ab_ac <= 0) {
            // Angle A is obtuse: use distance |AC|.
            return sqrt(dx1*dx1 + dy1*dy1);
        }

        // Dot product BA*BC is positive, when the angle is acute.
        AD<double> dot_product_ba_bc = dx*dx2 + dy*dy2;
        if (dot_product_ba_bc <= 0) {
            // Angle B is obtuse: use distance |BC|.
            return sqrt(dx2*dx2 + dy2*dy2);
        }

        // Both angles A and B are acute.
        // Compute distance to the line.
        return fabs(dy*x - dx*y + x2*y1 - y2*x1) / sqrt(dy*dy + dx*dx);
    }

    //
    // Compute distance from a point to a reference trajectory
    //
    AD<double> distanceToRefTrajectory(AD<double> x, AD<double> y) {

        // Get distance to the first trajectory segment.
        AD<double> distance = distanceToLineSegment(x, y,
            ref_x_[0], ref_y_[0], ref_x_[1], ref_y_[1]);

        for (unsigned i=2; i<ref_x_.size(); ++i) {

            // Distance to the next trajectory segment.
            AD<double> d = distanceToLineSegment(x, y,
                ref_x_[i-1], ref_y_[i-1], ref_x_[i], ref_y_[i]);

            if (d < distance)
                distance = d;
        }
        return distance;
    }

    //
    // Check whether a point lies beyond the reference trajectory
    //
    bool beyondTrajectory(AD<double> x, AD<double> y) {
        const int last = ref_x_.size() - 1;

        AD<double> x1 = ref_x_[last],   y1 = ref_y_[last];      // last point A
        AD<double> x2 = ref_x_[last-1], y2 = ref_y_[last-1];    // previous point B

        // Dot product AB*AC is negative, when the angle is obtuse.
        AD<double> dot_product_ab_ac = (x2 - x1)*(x - x1) +
                                       (y2 - y1)*(y - y1);
        return (dot_product_ab_ac < 0);
    }
};

//
// Implement MPC
//
MPC::MPC() {
    step_ = 0;
    round_ = 0;
    last_round_ = 0;
    init_x_ = 0.0;
    init_y_ = 0.0;
    trace_.open("mpc.trace");
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

void MPC::updatePred(vector<double> state0) {

    vector<double> next_state = state0;
    vector<double> next_actuator {steering_, throttle_};

    // allocate vectors
    pred_x_.resize(N_STEP);
    pred_y_.resize(N_STEP);

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

void MPC::updateRef(vector<double> x, vector<double> y,
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

void MPC::solve(vector<double> state0, vector<double> actuator0,
                vector<double> ptsx, vector<double> ptsy)
{
    //
    // print input data
    //
    cout << "time " << step_*LATENCY << "s        \r" << flush;
    trace_ << "--- step " << step_ << endl;
    trace_ << "    x = " << state0[0] << "  y = " << state0[1] <<
              "  psi = " << state0[2] << "  v = " << state0[3] << endl;
    trace_ << "    pts = ";
    for (int i=0; i<6; i++) {
        if (i > 0)
            trace_ << "  ";
        trace_ << "(" << ptsx[i] << " " << ptsy[i] << ")";
    }
    trace_ << endl;

    //
    // update the reference trajectory
    // the reference trajectory should be calculated using the measured state
    // vector since the way points and vehicle state were measured at the same
    // time prior to the current time.
    //
    updateRef(ptsx, ptsy, state0[0], state0[1], state0[2]);
    if (step_ == 0) {
        init_x_ = ptsx[0];
        init_y_ = ptsy[0];
    } else if (ptsx[0] == init_x_ && ptsy[0] == init_y_ && step_ > last_round_ + 20) {
        cout << "round " << round_ << " in " << (step_ - last_round_)*LATENCY << "s" << endl;
        round_++;
        last_round_ = step_;
    }
    step_++;

    //
    // estimate the current status to compensate the latency
    //
    vector<double> estimated_state0 = globalKinematic(state0, actuator0, LATENCY);

    //
    // set up the optimizer
    //
    typedef CPPAD_TESTVECTOR(double) Dvector;

    //
    // set variables
    //
    Dvector xi(2), xl(2), xu(2);

    // set variable boundaries
    xl[0] = -MAX_STEERING; xu[0] = MAX_STEERING;
    xl[1] = -MAX_THROTTLE; xu[1] = MAX_THROTTLE;

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

    // Print out the results
    trace_ << "solution status: " << solution.status << endl;
    trace_ << "Cost " << solution.obj_value << endl;
    trace_ << "optimized variables: ";
    for (unsigned i=0; i<solution.x.size(); ++i) {
        trace_ << solution.x[i] << "  ";
    }
    trace_ << endl;

    trace_ << "constraints: ";
    for (unsigned i=0; i<solution.g.size(); ++i) {
        trace_ << solution.g[i] << "  ";
    }
    trace_ << endl;

    // Check the solution status.
    // Possible values for the result status:
    //      0: not_defined,
    //      1: success,
    //      2: maxiter_exceeded,
    //      3: stop_at_tiny_step,
    //      4: stop_at_acceptable_point,
    //      5: local_infeasibility,
    //      6: user_requested_stop,
    //      7: feasible_point_found,
    //      8: diverging_iterates,
    //      9: restoration_failure,
    //      10: error_in_step_computation,
    //      11: invalid_number_detected,
    //      12: too_few_degrees_of_freedom,
    //      13: internal_error,
    //      14: unknown

    if (solution.status == CppAD::ipopt::solve_result<Dvector>::success) {
        // assign the optimized values
        steering_ = solution.x[0];
        throttle_ = solution.x[1];
    } else {
        // Cannot find optimal control.
        cout << "time " << step_*LATENCY << "s -- brake!\n";

        // Keep same steering angle and decelerate.
        if (state0[3] > 10)
            throttle_ = -MAX_THROTTLE;
        else if (state0[3] > 2)
            throttle_ = -MAX_THROTTLE/10;
        else
            throttle_ = 0;
    }

    // update the predicted trajectory
    updatePred(estimated_state0);

    //
    // print output data
    //
//throttle_value = 0.2;
    trace_ << "    steering = " << steering_ / MAX_STEERING <<
                "  throttle = " << throttle_ << endl;
}
