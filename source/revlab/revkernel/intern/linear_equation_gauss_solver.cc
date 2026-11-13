#include "KER_linear_equation_gauss_solver.hh"

GaussLinearEquationSolver::GaussLinearEquationSolver() : LinearEquationSolver(false) {
}

GaussLinearEquationSolver::~GaussLinearEquationSolver() {
}

bool GaussLinearEquationSolver::solve(const std::vector<std::vector<float>> &A_ref, const std::vector<float> &b, std::vector<float> &x) {
	std::vector<std::vector<float>> A = A_ref;

	x = b;

	const size_t n = x.size();
	for (size_t col = 0; col < n; col++) {
		size_t pivot = col;
		for (size_t r = col + 1; r < n; r++) {
			if (fabsf(A[r][col]) > fabsf(A[pivot][col]))
				pivot = r;
		}
		if (fabsf(A[pivot][col]) < 1e-12f)
			return false; /* singular */
		std::swap(A[pivot], A[col]);
		std::swap(x[pivot], x[col]);

		for (size_t r = col + 1; r < n; r++) {
			const float f = A[r][col] / A[col][col];
			if (f == 0.0f)
				continue;
			for (size_t c = col; c < n; c++)
				A[r][c] -= f * A[col][c];
			x[r] -= f * x[col];
		}
	}

	for (size_t i = n; i-- > 0;) {
		float sum = x[i];
		for (size_t c = i + 1; c < n; c++)
			sum -= A[i][c] * x[c];
		x[i] = sum / A[i][i];
	}

	return true;
}