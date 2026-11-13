#pragma once

#include <vector>

struct LinearEquationSolver;

/* -------------------------------------------------------------------- */
/** \name Linear Equation Solver Structure/Type
 * \{ */

struct LinearEquationSolver {
	LinearEquationSolver(bool supports_limits);
	virtual ~LinearEquationSolver();

	virtual bool solve(const std::vector<std::vector<float>> &A, const std::vector<float> &b, std::vector<float> &x);
	virtual bool solve_with_limits();

	bool supports_limits;
};

/** \} */
