/*
 * The Compass Search Solver (CS)
 */

typedef double compass_func(unsigned n, const double x[], int *not_feasible, void *arg);

void compass(
    compass_func  obj_func,         // objective function
    void         *arg,              // argument for objective function
    unsigned int  dim,              // dimension
    double        lb[],             // lower bounds
    double        ub[],             // upper bounds
    double        best_x[],         // starting point
    unsigned int *max_fevals,       // maximum number of function evaluations
    double        start_range,      // start range, say 0.1
    double        stop_range,       // stop range, like 1e-6
    double        reduction_coeff   // range reduction coefficient, typically 0.5
);
