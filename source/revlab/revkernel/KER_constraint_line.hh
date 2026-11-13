#pragma once

#include "LIB_math_vector_types.hh"

#include "KER_constraint.hh"
#include "KER_rigid_body.hh"

struct LineConstraint;

/* -------------------------------------------------------------------- */
/** \name Line Constraint Structure/Type
 * \{ */

struct LineConstraint : public Constraint {
	LineConstraint(RigidBody *body, float2 local, float2 global, float angle, float stiffness, float damping);
	virtual ~LineConstraint();

	void apply(PhysicsWorld *world, ConstraintMatrices *r_output);

	float2 local;
	float2 global;
	float angle;
	float stiffness;
	float damping;
};

/** \} */