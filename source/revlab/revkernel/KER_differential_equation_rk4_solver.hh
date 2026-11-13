#pragma once

#include "KER_differential_equation_solver.hh"

#include <array>
#include <vector>

struct RK4DifferentialEquationSolver;

/* -------------------------------------------------------------------- */
/** \name Differential Equation Solver Structure/Type
 * \{ */

// Fourth-Order Runge-Kutta
class RK4DifferentialEquationSolver : public DifferentialEquationSolver {
	struct RK4State {
		float3 value;
		float3 dvalue;
	};

	struct RK4Derivative {
		float3 dvalue;
		float3 ddvalue;
	};

public:
	RK4DifferentialEquationSolver();
	virtual ~RK4DifferentialEquationSolver();

	void init(PhysicsWorld *world, float dt);
	/** Returns true if the loop should stop running (stop calling #solve). */
	bool step(PhysicsWorld *world);
	void solve(PhysicsWorld *world);
	void end();

	float dt;
	int stage;
	std::vector<RK4State> y0;
	std::vector<std::array<RK4Derivative, 4>> k;
};

/** \} */