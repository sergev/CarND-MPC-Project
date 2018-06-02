/*
 * The Compass Search Solver (CS)
 */

typedef double compass_func(unsigned, const double[], double[], void *arg);

void compass(
    unsigned int  dim,              // dimension
    double        lb[],             // lower bounds
    double        ub[],             // upper bounds
    double        best_x[],         // starting point
    compass_func  obj_func,         // objective function
    unsigned int *max_fevals,       // maximum number of fitness evaluations
    double        start_range,      // start range, say 0.1
    double        stop_range,       // stop range, like 1e-6
    double        reduction_coeff,  // range reduction coefficient, typically 0.5
    void         *arg               // argument for objective function
);
