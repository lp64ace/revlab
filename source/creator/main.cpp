#include "LIB_units.hh"

#include "KER_crankshaft.hh"
#include "KER_cylinder_bank.hh"
#include "KER_differential_equation_euler_solver.hh"
#include "KER_linear_equation_gauss_solver.hh"
#include "KER_physics_generic.hh"
#include "KER_piston.hh"
#include "KER_rod.hh"

#include <chrono>
#include <iomanip>
#include <iostream>

int main(void) {
	GenericPhysicsWorld world(new EulerDifferentialEquationSolver, new GaussLinearEquationSolver);

	Crankshaft *crankshaft = KER_crankshaft_new(&world, float2(0.0f, 0.0f), 3.5f, 45e-4f, {RodJournal{LIB_unit_angle(90.0f, revlab::units::degrees), 0.043f}});
	CylinderBank *bank = KER_cylinder_bank_new(&world, float2(0.0f, 0.20f), LIB_unit_angle(90.0f, revlab::units::degrees), 0.04f, 20.0f);
	Piston *piston = KER_piston_new(&world, bank, float2(0.0f, 0.193f), 0.42, 0.0f);
	Rod *rod = KER_rod_new(&world, piston, crankshaft, 0, 0.55f, 14e-4f, 0.15f);

	float simulation_speed = 0.2f;

	auto prev = std::chrono::high_resolution_clock::now();
	while (true) {
		auto now = std::chrono::high_resolution_clock::now();
		float dt = std::chrono::duration<float>(now - prev).count();

		dt *= simulation_speed;
		if (dt < PhysicsWorld::delta_time) {
			continue;
		}

		world.process(dt);

		dt /= simulation_speed;
		fprintf(stdout, "%-16s [ X: %.3f Y: %.3f (%.3f) dX: %.3f dY: %.3f ]\n", "bank", bank->position.x, bank->position.y, bank->theta, bank->velocity.x, bank->velocity.y);
		fprintf(stdout, "%-16s [ X: %.3f Y: %.3f (%.3f) dX: %.3f dY: %.3f ]\n", "piston", piston->position.x, piston->position.y, piston->theta, piston->velocity.x, piston->velocity.y);
		fprintf(stdout, "%-16s [ X: %.3f Y: %.3f (%.3f) dX: %.3f dY: %.3f ]\n", "rod", rod->position.x, rod->position.y, rod->theta, rod->velocity.x, rod->velocity.y);
		fprintf(stdout, "%-16s [ X: %.3f Y: %.3f (%.3f) dX: %.3f dY: %.3f ]\n", "crankshaft", crankshaft->position.x, crankshaft->position.y, crankshaft->theta, crankshaft->velocity.x, crankshaft->velocity.y);
		fprintf(stdout, "fps : %.1f [%d]\n", 1.0f / dt, world.frame);

		crankshaft->v_theta = 1.0f;

		prev = now;
	}

	return 0;
}