/*
 * The DIRECT (DIviding RECTangles) derivative-free global optimization algorithm
 */

typedef double direct_func(unsigned n, const double x[], int *not_feasible, void *data);

int direct(
    direct_func   obj_func,         // objective function
    void         *f_data,           // argument for objective function
    int           dimension,        // dimension
    const double *lower_bounds,     // lower bounds
    const double *upper_bounds,     // upper bounds
    double       *x,                // starting point
    double       *minf,             // resulting function value
    int           max_feval,        // maximum number of function evaluations
    double        xtol_rel          // relative x tolerance (stop range), like 1e-3
);
