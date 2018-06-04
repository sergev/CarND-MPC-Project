/* Copyright (c) 2007-2014 Massachusetts Institute of Technology
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

typedef double direct_func(unsigned, const double[], double[], void *arg);

int direct_unscaled(
    int             n,
    direct_func     f,
    void           *f_data,
    const double   *lb,
    const double   *ub,
    double         *x,
    double         *minf,
    unsigned int   *max_fevals, // maximum number of function evaluations
    double          xtol_rel,   // relative x tolerance
    double          magic_eps,
    int             which_alg);

int direct(
    int             n,
    direct_func     f,
    void           *f_data,
    const double   *lb,
    const double   *ub,
    double         *x,
    double         *minf,
    unsigned int   *max_fevals, // maximum number of function evaluations
    double          xtol_rel,   // relative x tolerance
    double          magic_eps,
    int             which_alg);
