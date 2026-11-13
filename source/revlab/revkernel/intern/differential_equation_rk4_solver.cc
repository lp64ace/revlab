#include "KER_differential_equation_rk4_solver.hh"
#include "KER_rigid_body.hh"

RK4DifferentialEquationSolver::RK4DifferentialEquationSolver() {
}

RK4DifferentialEquationSolver::~RK4DifferentialEquationSolver() {
}

void RK4DifferentialEquationSolver::init(PhysicsWorld *world, float dt) {
	const size_t n = world->rigid_bodies.size();
	y0.resize(n);
	k.resize(n);

	/* The world has already snapshotted body -> shallow_copy. */
	for (size_t i = 0; i < n; i++) {
		const RigidBody *rb = world->rigid_bodies[i].shallow_copy;
		y0[i] = {float3(rb->position, rb->theta), float3(rb->velocity, rb->v_theta)};
	}

	this->stage = 0;
	this->dt = dt;
}

bool RK4DifferentialEquationSolver::step(PhysicsWorld *world) {
	return ++stage >= 4;
}

void RK4DifferentialEquationSolver::solve(PhysicsWorld *world) {
	for (size_t i = 0; i < world->rigid_bodies.size(); i++) {
		PhysicsRigidBodyStore &store = world->rigid_bodies[i];
		RigidBody *rb = store.shallow_copy;
		const float inv_M = rb->mass > 0.0f ? 1.0f / rb->mass : 0.0f;
		const float inv_I = rb->inertia > 0.0f ? 1.0f / rb->inertia : 0.0f;

		/* Derivative of the state at the point where forces were just evaluated. */
		RK4Derivative &d = k[i][stage];
		d.dvalue = float3(rb->velocity, rb->v_theta);
		d.ddvalue = float3(store.force * inv_M, store.torque * inv_I);

		const RK4State &s0 = y0[i];

		if (stage < 3) {
			const float h = (stage < 2) ? 0.5f * dt : dt;
			float3 v = s0.value + d.dvalue * h;
			float3 dv = s0.dvalue + d.ddvalue * h;
			rb->position = float2(v.x, v.y);
			rb->theta = v.z;
			rb->velocity = float2(dv.x, dv.y);
			rb->v_theta = dv.z;
		}
		else {
			const auto &K = k[i];
			const float h = dt / 6.0f;
			float3 v = s0.value + (K[0].dvalue + (K[1].dvalue + K[2].dvalue) * 2.0f + K[3].dvalue) * h;
			float3 dv = s0.dvalue + (K[0].ddvalue + (K[1].ddvalue + K[2].ddvalue) * 2.0f + K[3].ddvalue) * h;
			rb->position = float2(v.x, v.y);
			rb->theta = v.z;
			rb->velocity = float2(dv.x, dv.y);
			rb->v_theta = dv.z;
		}
	}
}

void RK4DifferentialEquationSolver::end() {
}
