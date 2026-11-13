#include "KER_linear_equation_solver.hh"

LinearEquationSolver::LinearEquationSolver(bool supports_limits) : supports_limits(supports_limits) {
}

LinearEquationSolver::~LinearEquationSolver() {
}

bool LinearEquationSolver::solve(const std::vector<std::vector<float>> &A, const std::vector<float> &b, std::vector<float> &x) {
	return false;
}

bool LinearEquationSolver::solve_with_limits() {
	return false;
}
