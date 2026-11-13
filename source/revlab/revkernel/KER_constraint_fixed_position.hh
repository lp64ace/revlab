#pragma once

#include "LIB_math_vector_types.hh"

#include "KER_constraint.hh"
#include "KER_rigid_body.hh"

struct FixedPositionConstraint;

/* -------------------------------------------------------------------- */
/** \name Fixed Position Constraint Structure/Type
 * \{ */

struct FixedPositionConstraint : public Constraint {
	FixedPositionConstraint(RigidBody *body, float2 local, float2 global, float stiffness, float damping);
	virtual ~FixedPositionConstraint();

	void apply(PhysicsWorld *world, ConstraintMatrices *r_output);

	float2 global;
	float2 local;
	float stiffness;
	float damping;
};

/** \} */
