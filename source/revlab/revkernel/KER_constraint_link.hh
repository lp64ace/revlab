#pragma once

#include "LIB_math_vector_types.hh"

#include "KER_constraint.hh"
#include "KER_rigid_body.hh"

struct LinkConstraint;

/* -------------------------------------------------------------------- */
/** \name Fixed Position Constraint Structure/Type
 * \{ */

struct LinkConstraint : public Constraint {
	LinkConstraint(RigidBody *body1, float2 local1, RigidBody *body2, float2 local2, float stiffness, float damping);
	virtual ~LinkConstraint();

	void apply(PhysicsWorld *world, ConstraintMatrices *r_output);

	float2 local[2];
	float stiffness;
	float damping;
};

/** \} */