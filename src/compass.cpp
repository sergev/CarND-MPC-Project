/*
 * The Compass Search Solver (CS)
 *
 * In the review paper by Kolda, Lewis, Torczon: 'Optimization by Direct Search: New Perspectives on Some Classical and
 * Modern Methods'
 * published in the SIAM Journal Vol. 45, No. 3, pp. 385-482 (2003), the following description of the compass search
 * algorithm is given:
 *
 * 'Davidon describes what is one of the earliest examples of a direct
 * search method used on a digital computer to solve an optimization problem:
 * Enrico Fermi and Nicholas Metropolis used one of the first digital computers,
 * the Los Alamos Maniac, to determine which values of certain theoretical
 * parameters (phase shifts) best fit experimental data (scattering cross
 * sections). They varied one theoretical parameter at a time by steps
 * of the same magnitude, and when no such increase or decrease in any one
 * parameter further improved the fit to the experimental data, they halved
 * the step size and repeated the process until the steps were deemed sufficiently
 * small. Their simple procedure was slow but sure, and several of us
 * used it on the Avidac computer at the Argonne National Laboratory for
 * adjusting six theoretical parameters to fit the pion-proton scattering data
 * we had gathered using the University of Chicago synchrocyclotron.
 * While this basic algorithm undoubtedly predates Fermi and Metropolis, it has remained
 * a standard in the scientific computing community for exactly the reason observed
 * by Davidon: it is slow but sure'.
 *
 * .. note::
 *
 *    This algorithm does not work for multi-objective problems, nor for stochastic problems.
 *
 * .. note::
 *
 *    The search range is defined relative to the box-bounds. Hence, unbounded problems
 *    will produce an error.
 *
 * .. note::
 *
 *    Compass search is a fully deterministic algorithms and will produce identical results if its evolve method is
 *    called from two identical populations.
 *
 * .. seealso::
 *
 *    Kolda, Lewis, Torczon: 'Optimization by Direct Search: New Perspectives on Some Classical and Modern Methods'
 *    published in the SIAM Journal Vol. 45, No. 3, pp. 385-482 (2003)
 *
 *    http://www.cs.wm.edu/~va/research/sirev.pdf
 */
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <stdbool.h>
#include "compass.h"

/*
 * Compass_search.
 * Continue until the search range becomes smaller than the defined stop_range.
 */
void compass(
    compass_func obj_func,          // objective function
    void        *arg,               // argument for objective function
    unsigned int dim,               // dimension
    double       lb[],              // lower bounds
    double       ub[],              // upper bounds
    double       best_x[],          // starting point
    unsigned int *max_fevals,       // maximum number of fitness evaluations
    double       start_range,       // start range, say 0.1
    double       stop_range,        // stop range, like 1e-6
    double       reduction_coeff    // range reduction coefficient, typically 0.5
) {
    assert(start_range > 0.0 && start_range <= 1.0);
    assert(stop_range <= 1.0 && stop_range < start_range);
    assert(reduction_coeff > 0.0 && reduction_coeff < 1.0);

    // Get value at starting point
    int not_feasible = 0;
    double best_f = obj_func(dim, best_x, &not_feasible, arg);

    // We need some auxiliary variables
    bool         flag     = false;
    unsigned int fevals   = 0u;
    double       newrange = start_range;

    while (newrange > stop_range && fevals <= *max_fevals) {
        flag = false;

        for (unsigned i = 0u; i < dim; i++) {
            double x_trial[dim];
            memcpy(x_trial, best_x, sizeof(x_trial));

            // move up
            x_trial[i] = best_x[i] + newrange * (ub[i] - lb[i]);

            // feasibility correction
            if (x_trial[i] > ub[i])
                x_trial[i] = ub[i];

            // objective function evaluation
            double f_trial = obj_func(dim, x_trial, &not_feasible, arg);

            fevals++;
            if (f_trial < best_f) {
                // accept
                //printf("    accept step up x%u, best = %g\n", i, f_trial);
                best_f = f_trial;
                memcpy(best_x, x_trial, sizeof(x_trial));
                flag = true;
                break;
            }

            // move down
            x_trial[i] = best_x[i] - newrange * (ub[i] - lb[i]);

            // feasibility correction
            if (x_trial[i] < lb[i])
                x_trial[i] = lb[i];

            // objective function evaluation
            f_trial = obj_func(dim, x_trial, &not_feasible, arg);

            fevals++;
            if (f_trial < best_f) {
                // accept
                //printf("    accept step down x%u, best = %g\n", i, f_trial);
                best_f = f_trial;
                memcpy(best_x, x_trial, sizeof(x_trial));
                flag = true;
                break;
            }
        }

        if (!flag) {
            // reduction
            newrange *= reduction_coeff;
            //printf("    reduction, newrange = %g\n", newrange);
        }
    }

    if (newrange <= stop_range) {
        //printf("Exit condition -- range: %g <= %g\n", newrange, stop_range);
    } else {
        //printf("Exit condition -- fevals: %u > %u\n", fevals, *max_fevals);
    }
    *max_fevals = fevals;
}
