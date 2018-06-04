//
// Model predictive controller
//
// Copyright (C) 2018 Serge Vakulenko
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
#include <iostream>
#include <assert.h>
#include <nlopt.h>
#include <math.h>
#include "mpc.h"
#include "compass.h"

const double LATENCY    = 0.1;      // latency of the data acquisition system (in seconds)
const double MAX_SPEED  = 90;       // maximum speed, miles per hour

// time step should be long enough to make the system be able to response timely
const double PRED_TIME_STEP     = 0.1;              // prediction time step in seconds
const int    N_STEP             = 15;               // number of prediction steps
const double MAX_STEERING       = 25 * M_PI/180;    // max 25 degrees
const double MAX_THROTTLE       = 1.0;
const double MAX_CTE            = 3.0;              // max cross track error

//
// transform coordinate from one coordinate system to the other
//
// @param X: coordinates (x, y) in the old coordinate system to be transformed
// @param X0: origin (x0, y0) of the new coordinate system in the old one
// @param psi: orientation of the new coordinate system with respect to the old one
//
vector<double> globalToCar(double px, double py, double px0, double py0, double psi)
{
    double dx     = px - px0;
    double dy     = py - py0;
    double cospsi = cos(psi);
    double sinpsi = sin(psi);

    // a rotation clock-wise about the origin
    vector<double> xy { dx*cospsi + dy*sinpsi,
                       -dx*sinpsi + dy*cospsi };
    return xy;
}

//
// The global kinematic model.
// Update the state of the car after time dt.
//
vector<double> globalKinematic(const vector<double> &state, const vector<double> &actuator, double dt)
{
    // distance between the front wheel and the vehicle center
    //const double LF = 2.67;
    const double LF = 2.8;

    // Conversion from MPH to meters per second.
    const double MPH_TO_METERS_PER_SEC = 0.44704;
    const double METERS_PER_SEC_TO_MPH = 2.23694;

    double px       = state[0];
    double py       = state[1];
    double psi      = state[2];
    double v        = state[3] * MPH_TO_METERS_PER_SEC;
    double steering = actuator[0];
    double throttle = actuator[1];

    // Acceleration in meters per sec^2 at maximum throttle.
    const double accel = (throttle >= 0) ? 6.0 : 20.0;

    vector<double> next_state(4);
    next_state[0] = px  + (v * cos(psi) * dt);
    next_state[1] = py  + (v * sin(psi) * dt);
    next_state[2] = psi - (v * steering/LF * dt);
    next_state[3] = (v + (throttle * accel * dt)) * METERS_PER_SEC_TO_MPH;
    return next_state;
}

//
// Compute distance from a point to a line segment
//
double distanceToLineSegment(
    double x,  double y,    // point C
    double x1, double y1,   // segment end A
    double x2, double y2)   // segment end B
{
    double dx = x2 - x1;
    double dy = y2 - y1;
    double dx1 = x1 - x;
    double dy1 = y1 - y;
    double dx2 = x2 - x;
    double dy2 = y2 - y;

    // Dot product AB*AC is positive, when the angle is acute.
    double dot_product_ab_ac = - dx*dx1 - dy*dy1;
    if (dot_product_ab_ac <= 0) {
        // Angle A is obtuse: use distance |AC|.
        return sqrt(dx1*dx1 + dy1*dy1);
    }

    // Dot product BA*BC is positive, when the angle is acute.
    double dot_product_ba_bc = dx*dx2 + dy*dy2;
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
double MPC::distanceToRefTrajectory(double x, double y)
{
    // Get distance to the first trajectory segment.
    double distance = distanceToLineSegment(x, y,
        ref_x_[0], ref_y_[0], ref_x_[1], ref_y_[1]);

    for (unsigned i=2; i<ref_x_.size(); ++i) {

        // Distance to the next trajectory segment.
        double d = distanceToLineSegment(x, y,
            ref_x_[i-1], ref_y_[i-1], ref_x_[i], ref_y_[i]);

        if (d < distance)
            distance = d;
    }
    return distance;
}

//
// Check whether a point lies beyond the reference trajectory
//
int MPC::beyondTrajectory(double x, double y)
{
    const int last = ref_x_.size() - 1;

    double x1 = ref_x_[last],   y1 = ref_y_[last];      // last point A
    double x2 = ref_x_[last-1], y2 = ref_y_[last-1];    // previous point B

    // Dot product AB*AC is negative, when the angle is obtuse.
    double dot_product_ab_ac = (x2 - x1)*(x - x1) +
                               (y2 - y1)*(y - y1);
    return (dot_product_ab_ac < 0);
}

double MPC::evaluatePenalty(vector<double> next_actuator)
{
    vector<double> next_state = state0_;
    double px0 = state0_[0];
    double py0 = state0_[1];
    double psi = state0_[2];
    double v   = state0_[3];

    double penalty  = 0;
    double ss_speed = 0.0;   // sum of square of speed
    double t        = 0;
    bool   off_road = false;

    neval_++;
    max_cte_ = 0.0;
    for (int i=0; i<N_STEP; ++i) {
        t += PRED_TIME_STEP;

        if (off_road) {
            penalty += 100;
        } else {
            // compute the next state
            next_state = globalKinematic(next_state, next_actuator, PRED_TIME_STEP);

            // transform from global coordinates system to car's coordinate system
            vector<double> xy = globalToCar(next_state[0], next_state[1], px0, py0, psi);

            if (beyondTrajectory(xy[0], xy[1])) {
                // Predicted point is beyond the reference trajectory
                //cout << "\nbeyond trajectory!" << endl;
                break;
            }

            // calculate the cross track error
            double cte = distanceToRefTrajectory(xy[0], xy[1]);

            if (cte > max_cte_) {
                max_cte_ = cte;
            }

            // penalty functions
            if (cte > MAX_CTE) {
                // Went off road.
                penalty += cte * cte * 100;

                if (!off_road) {
                    off_road = true;
                    next_actuator[0] = 0;
                    next_actuator[1] = 0;
                }
            } else if (i < N_STEP-1) {
                // Normal case.
                penalty += cte * cte;
            } else {
                // Last cte costs more.
                penalty += cte * cte * N_STEP;
            }

            ss_speed += next_state[3] * next_state[3];
            if (next_state[3] < 0) {
                // Don't drive backwards.
                penalty += ss_speed;
            }
        }
    }

    // speed control
    if (v < MAX_SPEED) {
        penalty -= (MAX_SPEED - v) * 1e-4 * ss_speed;
    }

    return penalty;
}

//
// Objective function.
//
double obj_func(unsigned n, const double *actuator, double *grad, void *arg)
{
    MPC *mpc = (MPC*) arg;
    vector<double> next_actuator { actuator[0], actuator[1] };

    return mpc->evaluatePenalty(next_actuator);
}

//
// Implement MPC
//
MPC::MPC()
{
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

void MPC::updatePred(vector<double> state0)
{
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
        vector<double> xy = globalToCar(next_state[0], next_state[1], state0[0], state0[1], state0[2]);

        // assign reference trajectory for each step
        pred_x_[i] = xy[0];
        pred_y_[i] = xy[1];
    }
}

void MPC::updateRef(vector<double> x, vector<double> y,
                    double px0, double py0, double psi)
{
    assert(x.size() == y.size());

    unsigned ref_length = x.size();

    // allocate vectors
    ref_x_.resize(ref_length);
    ref_y_.resize(ref_length);

    // transform from global coordinates system to car's coordinate system
    for (unsigned i=0; i<ref_length; ++i) {

        vector<double> xy = globalToCar(x[i], y[i], px0, py0, psi);

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

    double lower_bounds[2] = {-MAX_STEERING, -MAX_THROTTLE};
    double upper_bounds[2] = {MAX_STEERING,  MAX_THROTTLE};
    double actuator[2], cost;

    //
    // Global optimization.
    //
    nlopt_opt opt = nlopt_create(NLOPT_GN_ORIG_DIRECT, 2);

    // Set objective function.
    neval_ = 0;;
    nlopt_set_min_objective(opt, obj_func, this);

    // Specify bounds.
    nlopt_set_lower_bounds(opt, lower_bounds);
    nlopt_set_upper_bounds(opt, upper_bounds);

    // Stopping criteria, or. a relative tolerance on the optimization parameters.
    nlopt_set_xtol_rel(opt, 1e-3);
    nlopt_set_maxeval(opt, 500);

    // Perform the optimization, starting with some initial guess.
    actuator[0] = actuator0[0];
    actuator[1] = actuator0[1];
    state0_ = estimated_state0;
    nlopt_result status = nlopt_optimize(opt, actuator, &cost);
    nlopt_destroy(opt);

    trace_ << "    global status = " << status << endl;
    trace_ << "    " << neval_ << " global evals, cost = " << cost << "  max_cte = " << max_cte_ << endl;
    trace_ << "    result = " << actuator[0] << "  " << actuator[1] << endl;

    if ((status == NLOPT_SUCCESS || status == NLOPT_XTOL_REACHED) &&
        actuator[0] >= -MAX_STEERING && actuator[0] <= MAX_STEERING &&
        actuator[1] >= -MAX_THROTTLE && actuator[1] <= MAX_THROTTLE) {
        // result looks good
    } else {
        // Cannot find optimal control.
        cout << "time " << step_*LATENCY << "s -- global optimizer failed!\n";

        // Keep same steering angle and decelerate.
        actuator[0] = actuator0[0];
        actuator[1] = actuator0[1];
    }

    //
    // Local optimization.
    //
    // Stopping criteria, or. a relative tolerance on the optimization parameters.
    double   start_range = 0.1;
    double   stop_range  = 1e-3;
    unsigned maxeval     = 10000;

    neval_ = 0;
    state0_ = estimated_state0;
    compass(2, lower_bounds, upper_bounds, actuator, obj_func, &maxeval, start_range, stop_range, 0.5, this);
    status = NLOPT_SUCCESS;

    // Print out the results
    //trace_ << "    local status = " << status << endl;
    trace_ << "    " << neval_ << " local evals, cost = " << cost << "  max_cte = " << max_cte_ << endl;
    trace_ << "    result = " << actuator[0] << "  " << actuator[1] << endl;

    if ((status == NLOPT_SUCCESS || status == NLOPT_XTOL_REACHED || status == NLOPT_MAXEVAL_REACHED) &&
        actuator[0] >= -MAX_STEERING && actuator[0] <= MAX_STEERING &&
        actuator[1] >= -MAX_THROTTLE && actuator[1] <= MAX_THROTTLE) {
        // assign the optimized values

        steering_ = actuator[0];
        throttle_ = actuator[1];
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
    trace_ << "    steering = " << steering_ / MAX_STEERING <<
                "  throttle = " << throttle_ << endl;
}
