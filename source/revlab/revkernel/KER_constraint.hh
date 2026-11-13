#pragma once

#include "LIB_math_matrix_types.hh"

#include "KER_physics.hh"

struct Constraint;

/* -------------------------------------------------------------------- */
/** \name Constraint Structure/Type
 * \{ */

struct ConstraintMatrices {
	static constexpr int max_constraint_body_count = 2;

	/** The Jacobian matrix of the constraint. */
	float3x3 J[max_constraint_body_count];
	/** The derivative Jacobian matrix in respect to time of the constraint. */
	float3x3 dJ[max_constraint_body_count];
	/** The error of the constraint that needs to be addressed. */
	float3 C;

	/** Maximum allowed value per input (x, y, \theta) */
	float3 max;
	/** Minimum allowed value per input (x, y, \theta) */
	float3 min;

	/** Stiffness is applied per axis (for more flexibility). */
	float3 stiffness;
	/** Damping is applied per axis (for more flexibility). */
	float3 damping;
};

struct Constraint {
	Constraint(size_t bodies, size_t constraints);
	virtual ~Constraint();

	virtual void apply(PhysicsWorld *world, ConstraintMatrices *r_output) = 0;

	RigidBody *body[ConstraintMatrices::max_constraint_body_count];
	size_t bodies;
	size_t constraints;
};

/** \} */

/* -------------------------------------------------------------------- */
/** \name Constraint Common Utilities
 * \{ */

void KER_constraint_output_none(ConstraintMatrices *r_output);
void KER_constraint_output_no_limits(ConstraintMatrices *r_output);

/** \} */
