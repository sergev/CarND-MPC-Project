//
// Created by jun on 5/26/17.
// help functions
//
#ifndef MPC_UTILITIES_H
#define MPC_UTILITIES_H

//
// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
//
// @param s: input string
//
inline string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_last_of("}]");

    if (found_null != string::npos) {
        return "";
    }
    if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}

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
// evaluate a polynomial function
// a[0] + a[1]x + a[2]x^2 + ... + a[n-1]x^(n-1)
//
// @param a: coefficients of the polynomial
// @param x: variable
//
// @return result: evaluation of the polynomial function
//
template <class T>
inline T polyEval(Eigen::VectorXd a, T x) {
    T result = 0.0;

    for (int i = 0; i < a.size(); ++i) {
        result += a[i] * pow(x, i);
    }

    return result;
}

//
// solve a least square problem
//
// @param xvals: vector of x variables
// @param yvals: vector of y variables
// @param order: order of the polynomial fit
//
inline Eigen::VectorXd leastSquareFit(const Eigen::VectorXd& xvals,
                                      const Eigen::VectorXd& yvals, int order) {
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);

    Eigen::MatrixXd A(xvals.size(), order + 1);

    // assign the first column
    for (int i = 0; i < xvals.size(); ++i) {
        A(i, 0) = 1.0;
    }

    // assign the rest columns
    for (int j = 0; j < xvals.size(); ++j) {
        for (int i = 0; i < order; ++i) {
            A(j, i+1) = A(j, i) * xvals[j];
        }
    }

    // solve the least square problem using QR decomposition
    // Eigen::VectorXd result = A.householderQr().solve(yvals);  // fastest
    Eigen::VectorXd result = A.colPivHouseholderQr().solve(yvals);
    // Eigen::VectorXd result = A.fullPivHouseholderQr().solve(yvals);  // stablest

    return result;
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

#endif // MPC_UTILITIES_H
