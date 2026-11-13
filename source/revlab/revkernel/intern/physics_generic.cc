#include "KER_constraint.hh"
#include "KER_force_generator.hh"
#include "KER_physics_generic.hh"
#include "KER_rigid_body.hh"

using namespace revlab::math;

struct ConstraintRow {
	PhysicsRigidBodyStore *store[ConstraintMatrices::max_constraint_body_count];
	float3 J[ConstraintMatrices::max_constraint_body_count];
	float3 invM[ConstraintMatrices::max_constraint_body_count];
	size_t bodies;
};

GenericPhysicsWorld::GenericPhysicsWorld(DifferentialEquationSolver *de_solver, LinearEquationSolver *le_solver) {
	this->differential_equation_solver = de_solver;
	this->linear_equation_solver = le_solver;
}

GenericPhysicsWorld::~GenericPhysicsWorld() {
	delete this->differential_equation_solver;
	delete this->linear_equation_solver;
}

void GenericPhysicsWorld::update(float dt) {
	differential_equation_solver->init(this, dt);

	do {
		prepare();

		// process forces
		for (std::vector<PhysicsForceGeneratorStore>::iterator itr = force_generators.begin(); itr != force_generators.end(); itr++) {
			ForceGenerator *fg = itr->force_generator;
			fg->apply(this);
		}

		// process constraints
		size_t total_constraints = 0;
		for (std::vector<PhysicsConstraintStore>::iterator itr = constraints.begin(); itr != constraints.end(); itr++) {
			Constraint *c = itr->constraint;

			/** How many of these constraints are actualy used, maximum is 3 since we may have constraints on all 3 values (x, y, \theta). */
			total_constraints += c->constraints;
		}

		std::vector<std::vector<float>> A(total_constraints, std::vector<float>(total_constraints, 0.0f));
		std::vector<float> b(total_constraints, 0.0f);
		std::vector<ConstraintRow> rows(total_constraints);

		size_t index = 0;

		ConstraintMatrices output;
		for (std::vector<PhysicsConstraintStore>::iterator itr = constraints.begin(); itr != constraints.end(); itr++) {
			Constraint *c = itr->constraint;

			KER_constraint_output_none(&output);
			c->apply(this, &output);

			for (size_t subconstraint = 0; subconstraint < c->constraints; subconstraint++, index++) {
				ConstraintRow &row = rows[index];
				row.bodies = c->bodies;

				float rhs = 0.0f;
				for (size_t subbody = 0; subbody < c->bodies; subbody++) {
					const RigidBody *rb = c->body[subbody];
					PhysicsRigidBodyStore *store = get_rigid_body_store(rb);
					RigidBody *erb = store->shallow_copy;

					const float inv_M = erb->mass > 0.0f ? 1.0f / erb->mass : 0.0f;
					const float inv_I = erb->inertia > 0.0f ? 1.0f / erb->inertia : 0.0f;

					assert(store);

					float3 v(erb->velocity, erb->v_theta);
					float3 Jr = output.J[subbody][subconstraint];
					float3 dJr = output.dJ[subbody][subconstraint];
					float3 Fext = float3(store->force, store->torque);
					float3 invM = float3(inv_M, inv_M, inv_I);

					row.store[subbody] = store;
					row.J[subbody] = Jr;
					row.invM[subbody] = invM;

					rhs -= dot(dJr, v) + output.damping[subconstraint] * dot(Jr, v) + dot(Jr * invM, Fext);
				}
				rhs -= output.stiffness[subconstraint] * output.C[subconstraint];
				b[index] = rhs;
			}
		}

		/* A = J M^-1 J^T */
		for (size_t i = 0; i < total_constraints; i++) {
			for (size_t j = i; j < total_constraints; j++) {
				float sum = 0.0f;
				for (size_t k = 0; k < rows[i].bodies; k++) {
					for (size_t l = 0; l < rows[j].bodies; l++) {
						if (rows[i].store[k] == rows[j].store[l]) {
							sum += dot(rows[i].J[k] * rows[i].invM[k], rows[j].J[l]);
						}
					}
				}
				A[i][j] = sum;
				A[j][i] = sum;
			}
			A[i][i] += 1e-6f;
		}

		std::vector<float> x(total_constraints, 0.0f);

		bool solved = false;
		if (linear_equation_solver->supports_limits) {
			// solved |= linear_equation_solver->solve_with_limits(...);
		}
		if (!solved) {
			solved |= linear_equation_solver->solve(A, b, x);
		}

		assert(solved);

		for (size_t constraint = 0; constraint < total_constraints; constraint++) {
			for (size_t subbody = 0; subbody < rows[constraint].bodies; subbody++) {
				PhysicsRigidBodyStore *store = rows[constraint].store[subbody];
				const float3 o = rows[constraint].J[subbody] * x[constraint];
				store->force.x += o.x;
				store->force.y += o.y;
				store->torque += o.z;
			}
		}

		differential_equation_solver->solve(this);
	} while (!differential_equation_solver->step(this));

	differential_equation_solver->end();

	for (std::vector<PhysicsRigidBodyStore>::iterator itr = rigid_bodies.begin(); itr != rigid_bodies.end(); itr++) {
		RigidBody *rb = itr->body;
		RigidBody *erb = itr->shallow_copy;

		rb->position = erb->position;
		rb->velocity = erb->velocity;

		rb->theta = erb->theta;
		rb->v_theta = erb->v_theta;
	}
}
