#pragma once

#include "KER_linear_equation_solver.hh"

struct GaussLinearEquationSolver;

/* -------------------------------------------------------------------- */
/** \name Linear Equation Solver Structure/Type
 * \{ */

struct GaussLinearEquationSolver : public LinearEquationSolver {
	GaussLinearEquationSolver();
	virtual ~GaussLinearEquationSolver();

	virtual bool solve(const std::vector<std::vector<float>> &A, const std::vector<float> &b, std::vector<float> &x);

	bool supports_limits;
};

/** \} */